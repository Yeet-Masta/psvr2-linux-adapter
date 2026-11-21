# SPDX-License-Identifier: GPL-2.0
CFLAGS   ?= -O2 -Wall -Wextra
# The INS kernels match Sony's float results only without FMA contraction.
CFLAGS   += -ffp-contract=off
CPPFLAGS += -I. -I../../kernel
PREFIX   ?= /usr/local
LIBDIR   ?= $(PREFIX)/lib
INCDIR   ?= $(PREFIX)/include
PCDIR    ?= $(LIBDIR)/pkgconfig

all: libpsvr2.a libpsvr2.so psvr2-monitor

OBJS = libpsvr2.o psvr2_distortion.o psvr2_camera_model.o psvr2_led_solver.o psvr2_ins.o \
       sie/irtracker/psvr2_ins_core.o

%.o: %.c libpsvr2.h psvr2_lens_lut.h psvr2_led_solver.h psvr2_led_models.h psvr2_ins.h \
     sie/irtracker/psvr2_ins_core.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -fPIC -c -o $@ $<

libpsvr2.a: $(OBJS)
	$(AR) rcs $@ $^

libpsvr2.so: $(OBJS)
	$(CC) -shared -Wl,-soname,libpsvr2.so -o $@ $^ -lm

# The example only needs the public header, not the kernel uapi.
psvr2-monitor: psvr2-monitor.c libpsvr2.a
	$(CC) -I. $(CFLAGS) -o $@ psvr2-monitor.c libpsvr2.a -lm

test_led_solver: test_led_solver.c libpsvr2.a
	$(CC) -I. $(CFLAGS) -o $@ test_led_solver.c libpsvr2.a -lm

test_ins: test_ins.c libpsvr2.a
	$(CC) -I. $(CFLAGS) -o $@ test_ins.c libpsvr2.a -lm

check: test_led_solver test_ins
	./test_led_solver
	./test_ins

install: all
	install -Dm644 libpsvr2.h $(DESTDIR)$(INCDIR)/libpsvr2.h
	install -Dm644 libpsvr2.a $(DESTDIR)$(LIBDIR)/libpsvr2.a
	install -Dm755 libpsvr2.so $(DESTDIR)$(LIBDIR)/libpsvr2.so
	install -Dm644 libpsvr2.pc $(DESTDIR)$(PCDIR)/libpsvr2.pc

clean:
	$(RM) $(OBJS) libpsvr2.a libpsvr2.so psvr2-monitor test_led_solver test_ins

.PHONY: all install clean check
