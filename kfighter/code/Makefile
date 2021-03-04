.PHONY: all clean inc

CFLAGS=-g -ggdb -Wall -DKFIGHTER_SLOW=1 -DKFIGHTER_INTERNAL=1 -DKFIGHTER_LINUX=1 \
	-Wno-unused-function
LDFLAGS=-pthread -lX11 -lm -ldl
OBJDIR=../build
HEADERS=$(wildcard *.h)
SOURCE=$(wildcard *.cpp)

all: $(OBJDIR)/kfighter.so $(OBJDIR)/linux_kfighter

$(OBJDIR)/linux_kfighter: linux_kfighter.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(OBJDIR)/linux_kfighter $<

$(OBJDIR)/kfighter.so: kfighter.cpp $(SOURCE) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -shared -fPIC \
	-o $(OBJDIR)/kfighter.so $<

clean:
	rm -f $(OBJDIR)/linux_kfighter $(OBJDIR)/kfighter.so $(OBJDIR)/kfighter_temp.so

inc: $(OBJDIR)/kfighter.so
