#!/usr/bin/make -f

INCLUDES = -I./sensors

# Platform and optimization options

BASE_FLAGS = -Wall -Wextra -O2 -g \
	  $(INCLUDES) \
	  $(empty)
CFLAGS  = $(BASE_FLAGS) -std=gnu11
CXXFLAGS= $(BASE_FLAGS) -fno-rtti -fno-exceptions -std=gnu++11

EXE = test_so3

OBJS = main.o sensors/nanoTime.o

LIBS = -lm

all: $(EXE)

$(EXE) : $(OBJS) $(LIBS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LIBS)

clean:
	-rm -f $(OBJS) $(EXE)

.cpp.o:
	@echo g++ $<
	@$(CXX) $(CXXFLAGS) -c -o $@ $<

.c.o:
	@echo gcc $<
	@$(CC) $(CFLAGS) -c -o $@ $<

.s.o:
	@echo as $<
	@$(AS) $(ASFLAGS) -c -o $@ $<

