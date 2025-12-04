CC = gcc
CFLAGS = -Wall -Wextra -O2

TARGET = bmp280_test
SOURCES = bmp280_test.c
HEADERS = bmp280.h

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

.PHONY: all clean install
