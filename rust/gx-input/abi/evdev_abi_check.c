/* Compile-time check of the hand-written evdev ABI in gx-input/src/resync.rs
 * (InputAbsinfo, EVIOCGABS, EVIOCGKEY, KEY_BITMAP_BYTES) against the pinned
 * Linux 6.16 uapi headers: Debian linux-libc-dev 6.16.7-1, sha256
 * aa15f58cf9ec53d9c0d2330af2f523a1251de6f30428f2d3a780be3b0f811b1e, fetched by
 * tools/fetch_m0_scanout_headers.py. arm64 <asm/ioctl.h> is asm-generic.
 * Compile only (no link), e.g.:
 *   zig cc -target aarch64-linux-gnu -c -o /dev/null -nostdinc \
 *     -isystem <kernel-uapi>/usr/include -isystem <aarch64 libc>/usr/include \
 *     rust/gx-input/abi/evdev_abi_check.c
 * Any mismatch is a compile error. */
#include <stddef.h>
#include <linux/input.h>
_Static_assert(sizeof(struct input_absinfo) == 24, "size");
_Static_assert(_Alignof(struct input_absinfo) == 4, "align");
_Static_assert(offsetof(struct input_absinfo, value) == 0, "");
_Static_assert(offsetof(struct input_absinfo, minimum) == 4, "");
_Static_assert(offsetof(struct input_absinfo, maximum) == 8, "");
_Static_assert(offsetof(struct input_absinfo, fuzz) == 12, "");
_Static_assert(offsetof(struct input_absinfo, flat) == 16, "");
_Static_assert(offsetof(struct input_absinfo, resolution) == 20, "");
_Static_assert(KEY_MAX == 0x2ff && ABS_MAX == 0x3f, "");
_Static_assert((KEY_MAX + 1) / 8 == 96, "");
_Static_assert(EVIOCGABS(0) == 0x80184540u, "");
_Static_assert(EVIOCGABS(ABS_THROTTLE) == 0x80184546u, "");
_Static_assert(EVIOCGABS(ABS_HAT0Y) == 0x80184551u, "");
_Static_assert(EVIOCGABS(ABS_MAX) == 0x8018457fu, "");
_Static_assert(EVIOCGKEY(96) == 0x80604518u, "");
_Static_assert(ABS_X==0&&ABS_Y==1&&ABS_RZ==5&&ABS_THROTTLE==6&&ABS_HAT0X==0x10&&ABS_HAT0Y==0x11&&BTN_TRIGGER==0x120,"");
_Static_assert(SYN_DROPPED == 3, "");
int anchor;
