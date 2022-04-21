import math

def angle_trunc(a):
    while a < 0.0:
        a += math.pi * 2
    return a

def rad2deg(a):
    return 180 * a / math.pi

def deg2rad(a):
    return math.pi * a / 180

def offset(iterable):
    prev = None
    for elem in iterable:
        yield prev, elem
        prev = elem