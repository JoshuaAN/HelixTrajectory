import numpy as np

def solve_distinct_corners(x, y, theta, length, width):
    sin = np.sin(theta)
    cos = np.cos(theta)
    p0 = (x+(length/2)*cos-(width/2)*sin, y+(width/2)*cos+(length/2)*sin)
    p1 = (x-(length/2)*cos-(width/2)*sin, y+(width/2)*cos-(length/2)*sin)
    p2 = (x+(length/2)*cos+(width/2)*sin, y-(width/2)*cos+(length/2)*sin)
    p3 = (x-(length/2)*cos+(width/2)*sin, y-(width/2)*cos-(length/2)*sin)
    return [p0, p1, p2, p3]

def solve_corners(x, y, theta, length, width):
    p = solve_distinct_corners(x, y, theta, length, width)
    return [[p[0],p[1]],[p[0],p[2]],[p[1],p[3]],[p[2],p[3]]]