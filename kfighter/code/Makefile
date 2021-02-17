.PHONY: all clean

CFLAGS=-g -ggdb -Wall -DKFIGHTER_SLOW=1 -DKFIGHTER_INTERNAL=1 -DKFIGHTER_LINUX=1
LDFLAGS=-pthread -lX11 -ldl
OBJDIR=../build
HEADERS=$(wildcard *.h)
SOURCE=$(wildcard *.cpp)

$(OBJDIR)/linux_kfighter: linux_kfighter.cpp $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(OBJDIR)/linux_kfighter $<

$(OBJDIR)/kfighter: kfighter.cpp $(SOURCE) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(OBJDIR)/kfighter $<

all: $(OBJDIR)/kfighter $(OBJDIR)/linux_kfighter

clean:
	rm $(OBJDIR)/linux_kfighter $(OBJDIR)/kfighter

inc: $(OBJDIR)/kfighter
