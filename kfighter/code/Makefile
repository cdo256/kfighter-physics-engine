.PHONY: all clean

CFLAGS=-Wall -DKFIGHTER_SLOW=1 -DKFIGHTER_INTERNAL=1 -DKFIGHTER_LINUX=1
LDFLAGS=-incremental:no
OBJDIR=../build
HEADERS=$(wildcard *.h)
OBJECTS=$(wildcard *.cpp)

$(OBJDIR)/%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $<

$(OBJDIR)/linux_kfighter: linux_kfighter.o
	$(CC) $(CFLAGS) $* $<
	
$(OBJDIR)/kfighter: $(OBJECTS)
	$(CC) $(CFLAGS) $* $<

all: $(OBJDIR)/kfighter $(OBJDIR)/linux_kfighter 

clean:
	rm $(OBJDIR)/*

inc: $(OBJDIR)/kfighter
