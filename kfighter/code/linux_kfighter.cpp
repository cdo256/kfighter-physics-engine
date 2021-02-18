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

void linuxErrorMessage(char const* str) {
	fprintf(stderr, "Fatal error: %s\n", str);
}

void linuxResizePixmap(int width, int height) {

}

void linuxHandleEvent(XEvent ev) {
	switch (ev.type) {
	case NoExpose: {
		//NOOP
	} break;
	case Expose: {
		XSetForeground(display, gc, 0xFFFFFFFF);
		XFillRectangle(display, pm, gc, 0, 0, 800, 600);
		XSetForeground(display, gc, 0xFF0000FF);
		XFillRectangle(display, pm, gc, 20, 20, 10, 10);
		XDrawString(display, pm, gc, 10, 50, "hello", 5);
		XCopyArea(display, pm, win, gc, 0, 0, 800, 600, 0, 0);
	} break;
	case ResizeRequest: {
		XResizeRequestEvent e = ev.xresizerequest;
		linuxResizePixmap(e.width, e.height);
	} break;
	case ClientMessage: {
		if ((Atom)ev.xclient.data.l[0] == wmDeleteWindow) {
			running = false;
		}
	} break;
	default: {
		printf(%Unrecognised message: %i\n", ev.type);
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
	swa.colormap = XCreateColormap(display, root, vinfo.visual, AllocNone);
	win = XCreateWindow(
		display, root,
		0, 0, 800, 600, 1, 32, InputOutput,
		vinfo.visual, CWBackPixel | CWBorderPixel | CWColormap, &swa);
	XStoreName(display, win, "KFighter");
	wmDeleteWindow = XInternAtom(display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(display, win, &wmDeleteWindow, 1);
	XSelectInput(display, win, ExposureMask | KeyPressMask | ResizeRedirectMask);
	XMapWindow(display, win);
	XGCValues xgcvals = {0};
	gc = XCreateGC(display, win, 0, &xgcvals);
	pm = XCreatePixmap(display, win, 800, 600, 32);
	while (running) {
		XEvent e;
		XNextEvent(display, &e);
		linuxHandleEvent(e);
	}
	XCloseDisplay(display);
	return 0;
}