.PHONY: all clean

CFLAGS=-Wall -DKFIGHTER_SLOW=1 -DKFIGHTER_INTERNAL=1 -DKFIGHTER_LINUX=1
LDFLAGS=-incremental:no
OBJDIR=../build
HEADERS=$(wildcard *.h)
SOURCE=$(wildcard *.cpp)

$(OBJDIR)/linux_kfighter: linux_kfighter.cpp $(HEADERS)
	$(CC) $(CFLAGS) -o $(OBJDIR)/linux_kfighter $<
	
$(OBJDIR)/kfighter: kfighter.cpp $(SOURCE) $(HEADERS)
	$(CC) $(CFLAGS) -o $(OBJDIR)/kfighter $<

all: $(OBJDIR)/kfighter $(OBJDIR)/linux_kfighter

clean:
	rm $(OBJDIR)/linux_kfighter $(OBJDIR)/kfighter

inc: $(OBJDIR)/kfighter
