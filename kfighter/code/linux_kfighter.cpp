#include <stdlib.h>
#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

#include "kfighter_global.h"
#include "kfighter_maths.h"

global Display* display;
global Window win;
global GC gc;
global Atom wmDeleteWindow;
global bool running = true;
global int winWidth = 800, winHeight = 600;
global XImage* backBuffer;

void linuxErrorMessage(char const* str) {
	fprintf(stderr, "Fatal error: %s\n", str);
}

void linuxDrawPattern(XImage* b) {
	local_persist int offset = 0;
	char* line = b->data;
	for (int y = 0; y < b->height; y++) {
		for (int x = 0; x < b->width; x++) {
			((u32*)line)[x] = (x ^ y) | ((offset + x - y) << 8) | ((x | y) << 16);
		}
		line += b->bytes_per_line;
	}
	offset += 1;
}

void linuxResizeBackBuffer(int width, int height) {
	backBuffer->width = width;
	backBuffer->height = height;
	free(backBuffer->data);
	backBuffer->data = (char*)malloc(4 * width * height);
	if (backBuffer->data == NULL) {
		linuxErrorMessage("Could not resize back buffer.");
		exit(1);
	}
	backBuffer->bytes_per_line = 4 * width;
	winWidth = width;
	winHeight = height;
}

void linuxHandleEvent(XEvent ev) {
	switch (ev.type) {
	case NoExpose: {
		//NOOP
	} break;
	case Expose: {
	draw:
		linuxDrawPattern(backBuffer);
		XPutImage(display, win, gc, backBuffer, 0, 0, 0, 0, winWidth, winHeight);
	} break;
	case ConfigureNotify: {
		XConfigureEvent e = ev.xconfigure;
		linuxResizeBackBuffer(e.width, e.height);
		goto draw;
	} break;
	case ClientMessage: {
		if ((Atom)ev.xclient.data.l[0] == wmDeleteWindow) {
			running = false;
		}
	} break;
	default: {
		printf("Unrecognised message: %i\n", ev.type);
	} break;
	}
}

int main(int argc, char const* const* argv) {
	display = XOpenDisplay(NULL);
	if (!display) {
		linuxErrorMessage("Unable to open X11 display.");
		exit(1);
	}
	int screen = XDefaultScreen(display);
	Window root = XRootWindow(display, screen);
	XVisualInfo vinfo = {0};
	if (!XMatchVisualInfo(display, screen, 24, TrueColor, &vinfo)) {
		linuxErrorMessage("Could not get 32bit static color info.");
		exit(1);
	}
	//TODO: Should we specify more of these?
	XSetWindowAttributes swa = {0};
	swa.background_pixel = 0x00FFFFFF;
	swa.border_pixel = 0x00000000;
	swa.bit_gravity = NorthWestGravity;
	swa.event_mask = FocusChangeMask | KeyPressMask | KeyReleaseMask
		| ExposureMask | VisibilityChangeMask | StructureNotifyMask
		| ButtonMotionMask | ButtonPressMask | ButtonReleaseMask;
	swa.colormap = XCreateColormap(display, root, vinfo.visual, AllocNone);
	win = XCreateWindow(
		display, root,
		0, 0, winWidth, winHeight, 1, 24, InputOutput, vinfo.visual,
		CWBackPixel | CWBorderPixel | CWBitGravity | CWEventMask | CWColormap, &swa);
	XStoreName(display, win, "KFighter");
	wmDeleteWindow = XInternAtom(display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(display, win, &wmDeleteWindow, 1);
	XMapWindow(display, win);
	XGCValues xgcvals = {0};
	xgcvals.graphics_exposures = False;
	gc = XCreateGC(display, win, GCGraphicsExposures, &xgcvals);
	backBuffer = XCreateImage(display, vinfo.visual, 24, ZPixmap, 0,
		(char*)malloc(4 * winHeight * winWidth),
		winWidth, winHeight, 32, 4 * winWidth);
	if (!backBuffer) {
		linuxErrorMessage("Could not create back buffer.");
		exit(1);
	}
	while (running) {
		while (XPending(display)) {
			XEvent e;
			XNextEvent(display, &e);
			linuxHandleEvent(e);
		}
		linuxDrawPattern(backBuffer);
		XPutImage(display, win, gc, backBuffer, 0, 0, 0, 0, winWidth, winHeight);
	}
	XCloseDisplay(display);
	return 0;
}