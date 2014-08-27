#!/usr/bin/env python

from ctypes import CDLL, c_double, c_int, c_bool, byref

lib = CDLL("libuav_utils.so")


def spiral_sweep(start, end, width, interval, inward=True, tol=0.01):
    size = lib.spiral_size(c_double(start[0]), c_double(start[1]),
                           c_double(end[0]), c_double(end[1]),
                           c_double(width), c_double(interval))
    x = (c_double * size)()
    y = (c_double * size)()
    res = lib.spiral_sweep(c_double(start[0]), c_double(start[1]),
                           c_double(end[0]), c_double(end[1]),
                           c_double(width), c_double(interval),
                           c_bool(inward), c_double(tol),
                           byref(x), byref(y), size)
    if res != 0:
        return list()
    else:
        return [(x[i], y[i]) for i in range(size)]


def rect_sweep(start, end, width, interval):
    size = lib.rect_size(c_double(start[0]), c_double(start[1]),
                         c_double(end[0]), c_double(end[1]),
                         c_double(width), c_double(interval))
    x = (c_double * size)()
    y = (c_double * size)()
    res = lib.rect_sweep(c_double(start[0]), c_double(start[1]),
                         c_double(end[0]), c_double(end[1]),
                         c_double(width), c_double(interval),
                         byref(x), byref(y), size)
    if res != 0:
        return list()
    else:
        return [(x[i], y[i]) for i in range(size)]


if __name__ == '__main__':
    print spiral_sweep([0, 0], [20, 21], 3, 10)
    print rect_sweep([2, 2], [12, 12], 2, 5)
