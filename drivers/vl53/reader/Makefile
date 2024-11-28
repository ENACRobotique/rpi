CC := gcc

# Set to TRUE to enable logging functions.
LOG_ENABLE = FALSE

CORE_INCLUDE_PATHS = -I../uld-driver/inc
PLATFORM_INCLUDE_PATHS = -I../platform

BASE_CFLAGS = -Wall -Werror -Wno-missing-braces
CFLAGS_RELEASE = -Os -g0
#CFLAGS_RELEASE += -DSTMVL53L5CX_KERNEL

LIB_CORE_SOURCES =\
	$(wildcard ../uld-driver/src/*.c)

LIB_PLATFORM_SOURCES =\
	$(wildcard ../platform/*.c)

LIB_SOURCES := $(LIB_CORE_SOURCES) $(LIB_PLATFORM_SOURCES)
INCLUDE_PATH = $(CORE_INCLUDE_PATHS) $(PLATFORM_INCLUDE_PATHS)

CFLAGS = $(BASE_CFLAGS) $(CFLAGS_RELEASE) $(INCLUDE_PATH)

LIB_FLAGS = -lpigpiod_if2 -lrt

all:
	$(CC) $(CFLAGS) -o reader ./reader.c $(LIB_SOURCES) $(LIB_FLAGS)

clean:
	rm -f vl53_reader
