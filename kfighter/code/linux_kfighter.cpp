#include <stdlib.h>
#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

#include "kfighter_global.h"

global Display* display;
global Window win;
global GC gc;
global Atom wmDeleteWindow;
global bool running = true;
global Pixmap pm;
global int winWidth, winHeight;

void linuxErrorMessage(char const* str) {
	fprintf(stderr, "Fatal error: %s\n", str);
}

void linuxResizePixmap(int width, int height) {
	XFreePixmap(display, pm);
	pm = XCreatePixmap(display, win, width, height, 32);
	//XResizeWindow(display, win, width, height);
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
		int mx = winWidth/2;
		int my = winHeight/2;
		XSetForeground(display, gc, 0x00FFFFFF);
		XFillRectangle(display, win, gc, 0, 0, winWidth, winHeight);
		XSetForeground(display, gc, 0xFF0000FF);
		XFillRectangle(display, pm, gc, mx-10, my-10, 20, 20);
		XDrawString(display, pm, gc, mx-15, my+30, "hello", 5);
		//XCopyArea(display, pm, win, gc, 0, 0, winWidth, winHeight, 0, 0);
	} break;
	case ConfigureNotify: {
		XConfigureEvent e = ev.xconfigure;
		linuxResizePixmap(e.width, e.height);
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
	if (!XMatchVisualInfo(display, screen, 32, TrueColor, &vinfo)) {
		linuxErrorMessage("Could not get 32bit static color info.");
		exit(1);
	}
	//TODO: Should we specify more of these?
	XSetWindowAttributes swa = {0};
	swa.background_pixel = 0xFFFFFFFF;
	swa.border_pixel = 0x00000000;
	swa.bit_gravity = NorthWestGravity;
	swa.event_mask = FocusChangeMask | KeyPressMask | KeyReleaseMask
		| ExposureMask | VisibilityChangeMask | StructureNotifyMask
		| ButtonMotionMask | ButtonPressMask | ButtonReleaseMask;
	swa.colormap = XCreateColormap(display, root, vinfo.visual, AllocNone);
	win = XCreateWindow(
		display, root,
		0, 0, 800, 600, 1, 32, InputOutput, vinfo.visual,
		CWBackPixel | CWBorderPixel | CWBitGravity | CWEventMask | CWColormap, &swa);
	XStoreName(display, win, "KFighter");
	wmDeleteWindow = XInternAtom(display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(display, win, &wmDeleteWindow, 1);
	XMapWindow(display, win);
	XGCValues xgcvals = {0};
	xgcvals.graphics_exposures = False;
	gc = XCreateGC(display, win, GCGraphicsExposures, &xgcvals);
	winWidth = 800;
	winHeight = 600;
	pm = XCreatePixmap(display, win, 800, 600, 32);
	while (running) {
		XEvent e;
		XNextEvent(display, &e);
		linuxHandleEvent(e);
	}
	XCloseDisplay(display);
	return 0;
}