#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <x86intrin.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

#include "kfighter.h"
#include "kfighter_global.h"
#include "kfighter_maths.h"

struct LinuxGameCode {
	void* soHandle;
	game_update_and_render* updateAndRender;
	b32 isValid;
};

struct LinuxState {
	GC gc;
	int winWidth, winHeight;
	XImage* backBuffer;
	bool running;
	Window win;
	Atom wmDeleteWindow;
	Display* display;
	int xOffset, yOffset;
	bool key[256];
	LinuxGameCode gameCode;
	s32 loadCounter;
	GameMemory gameMemory;
	GameInput gameInput;
	GameOffscreenBuffer gameBuffer;
};

internal void
linuxErrorMessage(char const* str, ...) {
	va_list vl;
	va_start(vl, str);
	vfprintf(stderr, str, vl);
	fprintf(stderr, "\n");
	va_end(vl);
}

internal void*
linuxGetSymbolFromLibrary(void* handle, char const* name) {
	void* symbol = dlsym(handle, name);
	if (!symbol) {
		linuxErrorMessage("Could not find symbol %s in library: %s", name, dlerror());
		exit(1);
	}
	return symbol;
}

internal void*
linuxLoadLibrary(char const* filename) {
	char* buffer = (char*)malloc(strlen(filename)+3);
	strcpy(buffer, "./");
	strcat(buffer, filename);
	void* handle = dlopen(buffer, RTLD_NOW | RTLD_LOCAL);
	if (!handle)
	{
		linuxErrorMessage("Could not open library %s: %s", filename, dlerror());
		exit(1);
	}
	return handle;
}

internal void
linuxUnloadLibrary(void* handle) {
	dlclose(handle);
}

internal bool
linuxCopyFile(char const* srcFilename, char const* dstFilename) {
	char fileCopyBuffer[8192];
	bool success = false;
	int srcFd = open(srcFilename, O_RDONLY);
	if (srcFd >= 0) {
		int dstFd = open(dstFilename, O_WRONLY | O_CREAT);
		if (dstFd >= 0) {
			for (;;) {
				ssize_t readResult = read(srcFd, fileCopyBuffer, sizeof(fileCopyBuffer));
				if (readResult == 0) {
					success = true;
					break;
				}
				if (readResult < 0) break;
				ssize_t writeResult = write(dstFd, fileCopyBuffer, readResult);
				if (writeResult != readResult) break; 
			}
			close(dstFd);
		}
		close(srcFd);
	}
	return success;
}

internal char*
linuxGetExecutableDirectory() {
	char filenameBuffer[4096];
	ssize_t res = readlink("/proc/self/exe", filenameBuffer, sizeof(filenameBuffer));
	if (res == sizeof(filenameBuffer)) {
		linuxErrorMessage("Executable filename too long.");
		exit(1);
	} else if (res < 0) {
		linuxErrorMessage("Could not get executable filename.");
		exit(1);
	}
	filenameBuffer[res] = '\0';
	char* lastSlash = strrchr(filenameBuffer, '/');
	assert(lastSlash);
	lastSlash[1] = '\0';
	char* dir = (char*)malloc(strlen(filenameBuffer)+1);
	strcpy(dir, filenameBuffer);
	return dir;
}

internal LinuxGameCode
linuxLoadGameCode() {
	LinuxGameCode ret = {0};
	if (unlink("kfighter_temp.so") < 0) {
		linuxErrorMessage("Could not delete old kfighter_temp.so.");
	}
	if (!linuxCopyFile("kfighter.so", "kfighter_temp.so")) {
		linuxErrorMessage("Could not move file kfighter.so to kfighter_temp.so.");
		exit(1);
	}
	
	if (chmod("kfighter_temp.so", 0755) < 0) {
		linuxErrorMessage("Could not set kfighter_temp.so file permissions.");
		exit(1);
	}
	ret.soHandle = linuxLoadLibrary("kfighter_temp.so");
	ret.updateAndRender = (game_update_and_render*)
		linuxGetSymbolFromLibrary(ret.soHandle, "gameUpdateAndRender");
	ret.isValid = (ret.updateAndRender != NULL);
	if (!ret.isValid) {
		ret.updateAndRender = gameUpdateAndRenderStub;
	}
	return ret;
}

internal void
linuxUnloadGameCode(LinuxGameCode *code) {
	if (code->soHandle) {
		linuxUnloadLibrary(code->soHandle);
		code->soHandle = NULL;
	}
	code->isValid = false;
	code->updateAndRender = gameUpdateAndRenderStub;
}

internal void
linuxDrawPattern(modified XImage* b, int xOffset, int yOffset) {
	char* line = b->data;
	for (int y = 0; y < b->height; y++) {
		for (int x = 0; x < b->width; x++) {
			((u32*)line)[x] = (x ^ y) | ((xOffset + x - y) << 8)
				| ((x | (y + yOffset)) << 16);
		}
		line += b->bytes_per_line;
	}
}

internal void
linuxUpdateGameBuffer(out GameOffscreenBuffer* gameBuffer,
		in XImage* xBuffer) {
	gameBuffer->bitmapMemory = xBuffer->data;
	gameBuffer->width = xBuffer->width;
	gameBuffer->height = xBuffer->height;
	gameBuffer->pitch = xBuffer->bytes_per_line;
	gameBuffer->bytesPerPixel = 4;
}

internal void
linuxResizeBackBuffer(modified LinuxState* state, int width, int height) {
	state->backBuffer->width = width;
	state->backBuffer->height = height;
	free(state->backBuffer->data);
	state->backBuffer->data = (char*)malloc(4 * width * height);
	if (state->backBuffer->data == NULL) {
		linuxErrorMessage("Could not resize back buffer.");
		exit(1);
	}
	state->backBuffer->bytes_per_line = 4 * width;
	state->winWidth = width;
	state->winHeight = height;
	linuxUpdateGameBuffer(&state->gameBuffer, state->backBuffer);
}

internal bool
linuxInitGameMemory(out GameMemory* memory) {
	memory->permanentStorageSize = MEGABYTES(4);
	memory->transientStorageSize = MEGABYTES(16);
	memory->permanentStorage = calloc(1, memory->permanentStorageSize);
	memory->transientStorage = calloc(1, memory->transientStorageSize);
	if (!memory->permanentStorage || !memory->transientStorage) {
		free(memory->permanentStorage);
		free(memory->transientStorage);
		return false;
	}
	return true;
}

internal bool
linuxInitGameInput(out GameInput* input) {
	//TODO
	return false;
}

internal bool
linuxInitState(out LinuxState* state) {
	if (!linuxInitGameMemory(&state->gameMemory)) {
		linuxErrorMessage("Could not allocate game memory.");
	}
	char* dir = linuxGetExecutableDirectory();
	chdir(dir);
	free(dir);
	state->display = XOpenDisplay(NULL);
	state->running = true;
	if (!state->display) {
		linuxErrorMessage("Unable to open X11 display.");
		return false;
	}
	state->winWidth = 800;
	state->winHeight = 600;
	int screen = XDefaultScreen(state->display);
	Window root = XRootWindow(state->display, screen);
	XVisualInfo vinfo = {0};
	if (!XMatchVisualInfo(state->display, screen, 24, TrueColor, &vinfo)) {
		linuxErrorMessage("Could not get 32bit static color info.");
		return false;
	}
	XSetWindowAttributes swa = {0};
	swa.background_pixel = 0x00FFFFFF;
	swa.border_pixel = 0x00000000;
	swa.bit_gravity = NorthWestGravity;
	swa.event_mask = FocusChangeMask | KeyPressMask | KeyReleaseMask
		| ExposureMask | VisibilityChangeMask | StructureNotifyMask
		| ButtonMotionMask | ButtonPressMask | ButtonReleaseMask;
	swa.colormap = XCreateColormap(state->display, root, vinfo.visual, AllocNone);
	state->win = XCreateWindow(
		state->display, root,
		0, 0, state->winWidth, state->winHeight, 1, 24, InputOutput, vinfo.visual,
		CWBackPixel | CWBorderPixel | CWBitGravity | CWEventMask | CWColormap, &swa);
	XStoreName(state->display, state->win, "KFighter");
	state->wmDeleteWindow = XInternAtom(state->display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(state->display, state->win, &state->wmDeleteWindow, 1);
	XMapWindow(state->display, state->win);
	XGCValues xgcvals = {0};
	xgcvals.graphics_exposures = False;
	state->gc = XCreateGC(state->display, state->win, GCGraphicsExposures, &xgcvals);
	state->backBuffer = XCreateImage(state->display, vinfo.visual, 24, ZPixmap, 0,
		(char*)malloc(4 * state->winHeight * state->winWidth),
		state->winWidth, state->winHeight, 32, 4 * state->winWidth);
	if (!state->backBuffer) {
		linuxErrorMessage("Could not create back buffer.");
		return false;
	}
	state->gameCode = linuxLoadGameCode();
	state->loadCounter = 0;
	return true;
}

internal void
linuxRedrawWindow(modified LinuxState* state) {
	XPutImage(state->display, state->win, state->gc,
		state->backBuffer, 0, 0, 0, 0, state->winWidth, state->winHeight);
}

internal void
linuxHandleEvent(XEvent ev, modified LinuxState* state) {
	switch (ev.type) {
	case NoExpose: {
		//NOOP
	} break;
	case Expose: {
		linuxRedrawWindow(state);
	} break;
	case ConfigureNotify: {
		XConfigureEvent e = ev.xconfigure;
		linuxResizeBackBuffer(state, e.width, e.height);
		linuxRedrawWindow(state);
	} break;
	case ClientMessage: {
		if ((Atom)ev.xclient.data.l[0] == state->wmDeleteWindow) {
			state->running = false;
		}
	} break;
	case FocusOut: {
		for (int i = 0; i < 256; i++) {
			state->key[ev.xkey.keycode] = false;
		}
	} break;
	case KeyPress: case KeyRelease: {
		XKeyEvent e = ev.xkey;
		bool isDown = (ev.type == KeyPress);
		if (e.keycode < 256) {
			state->key[e.keycode] = isDown;
		} 
	} break;
	default: {
		printf("Unrecognised message: %i\n", ev.type);
	} break;
	}
}

internal f32
ComputeClockInterval(struct timespec start, struct timespec end) {
	return (f32)(end.tv_sec - start.tv_sec) + ((f32)(end.tv_nsec - start.tv_nsec) * 1e-9f);
}

int
main(int argc, char const* const* argv) {
	LinuxState state = {0};
	
	if (!linuxInitState(&state)) {
		return 1;
	}

	struct timespec lastCounter;
	clock_gettime(CLOCK_MONOTONIC, &lastCounter);
	u64 lastCycleCount = __rdtsc();

	while (state.running) {
		while (XPending(state.display)) {
			XEvent e;
			XNextEvent(state.display, &e);
			linuxHandleEvent(e, &state);
		}
		if (state.loadCounter++ > 120) {
			linuxUnloadGameCode(&state.gameCode);
			state.gameCode = linuxLoadGameCode();
			state.loadCounter = 0;
		}
		int speed = 1;
		if (state.key[111]) state.yOffset -= speed;
		if (state.key[116]) state.yOffset += speed;
		if (state.key[113]) state.xOffset -= speed;
		if (state.key[114]) state.xOffset += speed;
		state.gameCode.updateAndRender(0.f, 4,
			&state.gameMemory, &state.gameInput,
			&state.gameBuffer);
		linuxRedrawWindow(&state);
		
		struct timespec endCounter;
		clock_gettime(CLOCK_MONOTONIC, &endCounter);
		u64 endCycleCount = __rdtsc();
		f32 timeElapsed = ComputeClockInterval(lastCounter, endCounter);
		printf("%3.2fms/f, %3.1ff/s, %2.2fMc/f\n",
			timeElapsed*1e3f, 1.f/timeElapsed,
			(endCycleCount - lastCycleCount) * 1e-6f);
		lastCycleCount = endCycleCount;
		lastCounter = endCounter;
	}
	XCloseDisplay(state.display);
	return 0;
}
