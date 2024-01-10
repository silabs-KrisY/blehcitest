TARGET ?= blehcitest
CC = gcc
LD = ld
C_SRC = blehcitest.c
CFLAGS=-g -Wall -Wextra $(shell pkg-config --libs --cflags bluez)
EXEDIR = exe

$(EXEDIR)/$(TARGET): $(C_SRC)
	mkdir -p $(EXEDIR)
	$(CC) $(DEBUG) -o $@ $^ $(CFLAGS)

debug: DEBUG = -DDEBUG

debug: $(EXEDIR)/$(TARGET)

clean:
	rm -f $(EXEDIR)/$(TARGET)
