CC = gcc
CFLAGS = -Wall -Wextra -O2 -pthread
TARGET = hmisrv
SRC = hmisrv.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

.PHONY: all clean install
