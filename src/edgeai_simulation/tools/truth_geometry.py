"""Planar transforms for diagnostics and explicitly enabled truth navigation tests."""
import math


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def planar(p, q):
    return (p.x, p.y, yaw(q))
