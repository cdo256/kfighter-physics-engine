#include <stdlib.h>
#include <stdio.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>

void linuxErrorMessage(char const* str) {
	fprintf(stderr, "%s", str);
}

int main(int argc, char const* const* argv) {
	Display* display = 0;
	display = XOpenDisplay(NULL);
	if (!display) {
		linuxErrorMessage("Unable to open X11 display.");
		exit(1);
	}
	int screen = XDefaultScreen(display);
	Window root = XRootWindow(display, screen);
	XSetWindowAttributes swa = {0};
	swa.background_pixel = 0x00FF00FF;
	Window win = XCreateWindow(
		display, root,
		10, 10, 200, 300, 5, CopyFromParent, InputOutput,
		CopyFromParent, CWBackPixel, &swa);
	XSelectInput(display, win, ExposureMask | KeyPressMask);
	XMapWindow(display, win);
	GC gc = XDefaultGC(display, screen);
	while (1) {
		XEvent e;
		XNextEvent(display, &e);
		if (e.type == Expose) {
			XFillRectangle(display, win, gc, 20, 20, 10, 10);
			XDrawString(display, win, gc, 10, 50, "hello", 5);
		} else if (e.type == KeyPress) break;
	}
	XCloseDisplay(display);
	return 0;
}