#include <stdlib.h>
#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

#include "kfighter_global.h"
#include "kfighter_maths.h"

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
};

void linuxErrorMessage(char const* str) {
	fprintf(stderr, "Fatal error: %s\n", str);
}

void linuxDrawPattern(XImage* b, int xOffset, int yOffset) {
	char* line = b->data;
	for (int y = 0; y < b->height; y++) {
		for (int x = 0; x < b->width; x++) {
			((u32*)line)[x] = (x ^ y) | ((xOffset + x - y) << 8) | ((x | (y + yOffset)) << 16);
		}
		line += b->bytes_per_line;
	}
}

void linuxResizeBackBuffer(LinuxState* state, int width, int height) {
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
}

bool linuxInitState(LinuxState* state) {
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
	return true;
}

void linuxRedrawWindow(LinuxState* state) {
	linuxDrawPattern(state->backBuffer, state->xOffset, state->yOffset);
	XPutImage(state->display, state->win, state->gc,
		state->backBuffer, 0, 0, 0, 0, state->winWidth, state->winHeight);
}

void linuxHandleEvent(XEvent ev, LinuxState* state) {
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

int main(int argc, char const* const* argv) {
	LinuxState state = {0};
	if (!linuxInitState(&state)) {
		return 1;
	}
	while (state.running) {
		while (XPending(state.display)) {
			XEvent e;
			XNextEvent(state.display, &e);
			linuxHandleEvent(e, &state);
		}
		int speed = 1;
		if (state.key[111]) state.yOffset -= speed;
		if (state.key[116]) state.yOffset += speed;
		if (state.key[113]) state.xOffset -= speed;
		if (state.key[114]) state.xOffset += speed;
		linuxRedrawWindow(&state);
	}
	XCloseDisplay(state.display);
	return 0;
}
