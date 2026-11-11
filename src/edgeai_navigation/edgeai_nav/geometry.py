"""Planar transforms shared by localization and task geometry."""
import math

def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return [a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]),math.cos(a[2]+b[2]))]

def relative(a,b):
    c,s=math.cos(a[2]),math.sin(a[2]);dx,dy=b[0]-a[0],b[1]-a[1]
    return [c*dx+s*dy,-s*dx+c*dy,math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))]
