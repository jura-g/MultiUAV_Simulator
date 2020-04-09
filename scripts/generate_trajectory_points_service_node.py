#!/usr/bin/env python

from math import sqrt, atan2, acos, pi
import rospy
from graupner_serial.srv import BuildingDescriptor

MIN_HEIGHT = 1.0

class GenerateTrajectory:
    def __init__(self):
        self.service = rospy.Service('generate_trajectory_points_service', \
                                      BuildingDescriptor, \
                                      generate_trajectory)

        print("generate_trajectory_service_node initialized")

'''
DESCRIPTION:
    Linear interpolation between given points with resolution res
INPUT:
    x   - list of x coordinates
    y   - list of y coordinates
    res - resolution (spatial)
OUTPUT:
    interp_x - list of interpolated x coordinates
    interp_y - list of interpolated y coordinates
'''
def interpolate(x, y, res):
    interp_x = []; interp_y = []

    for i in range(len(x)-1):
        vec_length = sqrt( (x[i+1]-x[i])**2 + (y[i+1]-y[i])**2 )
        unit_vec = [(x[i+1]-x[i])/vec_length, (y[i+1]-y[i])/vec_length]

        tmp_vec_length = 0; tmp_x = x[i]; tmp_y = y[i]
        while tmp_vec_length < vec_length:
            interp_x.append(tmp_x)
            interp_y.append(tmp_y)

            tmp_x += res*unit_vec[0]
            tmp_y += res*unit_vec[1]
            tmp_vec_length += res

    print(vec_length)
    print(x)
    print(len(x))
    print(y)
    print(len(y))

    end = len(x)-1
    vec_length = sqrt( (x[0]-x[end])**2 + (y[0]-y[end])**2 )
    unit_vec = [(x[0]-x[end])/vec_length, (y[0]-y[end])/vec_length]

    tmp_vec_length = 0; tmp_x = x[end]; tmp_y = y[end]
    while tmp_vec_length < vec_length:
        interp_x.append(tmp_x)
        interp_y.append(tmp_y)

        tmp_x += res*unit_vec[0]
        tmp_y += res*unit_vec[1]
        tmp_vec_length += res

    return interp_x, interp_y


'''
DESCRIPTION:
    Intersections of two circles
INPUTS:
    sx1, sy1, sx2, sy2 - center points of two circles
    r1, r2             - radius of circles
OUTPUTS:
    x1, y1, x2, y2 - intersection points
'''
def intersections(sx1, sy1, sx2, sy2, r1, r2):
    R = sqrt( (sx2-sx1)**2 + (sy2-sy1)**2 )

    x1 = 1./2*(sx1+sx2) + (r1**2-r2**2)/(2*R**2)*(sx2-sx1) + 1./2*sqrt(2*(r1**2+r2**2)/R**2 - (r1**2-r2**2)**2/R**4 - 1)*(sy2-sy1)
    x2 = 1./2*(sx1+sx2) + (r1**2-r2**2)/(2*R**2)*(sx2-sx1) - 1./2*sqrt(2*(r1**2+r2**2)/R**2 - (r1**2-r2**2)**2/R**4 - 1)*(sy2-sy1)

    y1 = 1./2*(sy1+sy2) + (r1**2-r2**2)/(2*R**2)*(sy2-sy1) + 1./2*sqrt(2*(r1**2+r2**2)/R**2 - (r1**2-r2**2)**2/R**4 - 1)*(sx1-sx2)
    y2 = 1./2*(sy1+sy2) + (r1**2-r2**2)/(2*R**2)*(sy2-sy1) - 1./2*sqrt(2*(r1**2+r2**2)/R**2 - (r1**2-r2**2)**2/R**4 - 1)*(sx1-sx2)

    return x1, y1, x2, y2


'''
DESCRIPTION:
    Angle of circle arc when going from the first point (x1, y1) on the circle 
    to the second point (x2, y2) on the circle with center at (sx, sy) in COUNTER CLOCKWISE direction.
INPUT:
    x1, y1, x2, y2 - points on the circle
    sx, sy         - circle center
OUTPUT:
    alpha - angle of circle art when going from first point to the other in counter clockwise direction
'''
def getAngle_ccw(x1, y1, x2, y2, sx, sy):
    x1 = x1 - sx; y1 = y1 - sy
    x2 = x2 - sx; y2 = y2 - sy

    alpha1 = atan2(y1, x1)
    alpha2 = atan2(y2, x2)

    alpha = alpha2 - alpha1
    if alpha < 0:
        alpha = 2*pi + alpha
    
    return alpha


'''
DESRIPTION:
    Find the first intersection of circles with centers (sx1, sy1), (sx2, sy2) and radius r1 and r2
    when going from point (last_x, last_y) over circle points with center in (sx1, sy1)
    in counter clockwise direction
INPUT:
    sx1, sy1, sx2, sy2 - centers of two circles
    r1, r2             - radius of circles
    last_x, last_y     - point from which to start (last intersection)
OUTPUT:
    x_inter, y_inter   - chosen intersection
'''
def intersection_ccw(sx1, sy1, sx2, sy2, r1, r2, last_x, last_y):
    x1, y1, x2, y2 = intersections(sx1, sy1, sx2, sy2, r1, r2)

    angle1 = getAngle_ccw(last_x, last_y, x1, y1, sx1, sy1)
    angle2 = getAngle_ccw(last_x, last_y, x2, y2, sx1, sy1)

    if angle1 < angle2:
        x_inter = x1; y_inter = y1
    else:
        x_inter = x2; y_inter = y2

    return x_inter, y_inter


'''
DESCRIPTION:
    Calculate length of the circle arc over secant of length d in circle with radius r
    Using cos theorem
INPUT:
    d - length of the secant
    r - radius of the circle
OUTPUT:
    x - length of the circle arc
'''
def arc_over_secant(d, r):
    alpha = acos( (2.*r**2 - d**2) / (2*r**2) )
    x = r*alpha
    return x


'''
DESCRIPTION:
    Generate trajectory points given two points on a circle, circle center (point on the building),
    circle radius (distance from the building - dist), path already passed from last trajectory point (s)
    and resolution (distance between two trajectory points)
INPUT:
    x1, y1, x2, y2 - points on the circle (trajectory follows counter clockwise direction)
    sx, sy         - circle center (point on the building)
    s              - path already passed from last trajectory point
    res            - resolution - distance between two trajectory points)
OUTPUT:
    traj_xs, traj_ys - list of trajectory points for that segment
    s                - path passed from last trajectory point to the second intersection (x2, y2)
'''
def make_traj_points(x1, y1, x2, y2, sx, sy, s, res, dist):
    new_xs = []; new_ys = []

    angle = getAngle_ccw(x1, y1, x2, y2, sx, sy)
    l = dist*angle
    while l > arc_over_secant(res-s, dist):
        # small circle with radius res-s and center in (x1, y1) -> moving forward (in ccw direction)
        #                                                          over fiven circle arc
        x1, y1 = intersection_ccw(sx, sy, x1, y1, dist, res-s, x1, y1)
        new_xs.append(x1)
        new_ys.append(y1)
        
        angle = getAngle_ccw(x1, y1, x2, y2, sx, sy)
        l = dist*angle
        s = 0
    
    new_s = s+l

    return new_xs, new_ys, new_s


'''
DESCRIPTION:
    Generates all trajectory points
INPUT:
    x, y    - list of points of the building (needed to interpolate)
    height  - height of the building
    dist    - distance of trajectory from the building
    res     - resolution of trajectory points (distance between two trajectory points)
    res_building_points - resolution of interpolated building points
    res_height - resolution of height levels of trajectory (same trajectory, different height)
OUTPUT:
    traj_x, traj_y, traj_z, traj_yaw - trajectory points
'''
def generate_trajectory(req):
    x = req.x; y = req.y; height = req.height
    dist = req.dist
    res = req.res; res_building_points = req.res_building_points; res_height = req.res_height

    x, y = interpolate(x, y, res_building_points)

    traj_x = []; traj_y = []
    tmp_x, tmp_y, tmp2_x, tmp2_y = intersections(x[0], y[0], x[1], y[1], dist, dist)    # random point random intersection

    # Points of the trajectory are always dist m apart from nearest point of the building
    # s - distance already passed on the way to the next trajectory point (trajectory points are distanced by res m)
    # minAngleIndex - index of a point with intersection with minimal angle(ccw) with respect to the last intersection
    #               - some of the points following last one maybe should be skipped because they don't satisfy 
    #                   initial condition, taht is in case of non convex edges (look at the example)
    #               - checking for the following N/4 points, where N is total number of points
    # minAngle - minimal angle(ccw) with respect to the last intersection
    start_inter_x = tmp_x; start_inter_y = tmp_y
    end_inter_x = 0; end_inter_y = 0
    s = 0; i = 0; minAngleIndex = 0
    while minAngleIndex >= i:
        i = minAngleIndex
        minAngle = 3*pi     # inf
        
        cnt = 0; j = i; check_pts = len(x)/4
        while cnt < check_pts:
            # j = j+1;
            j = (j+1) % len(x)
            cnt = cnt+1
            
            if sqrt( (x[i]-x[j])**2 + (y[i]-y[j])**2 ) > 2*dist:
                continue
            
            [inter_x, inter_y] = intersection_ccw(x[i], y[i], x[j], y[j], dist, dist, start_inter_x, start_inter_y)
            angle = getAngle_ccw(start_inter_x, start_inter_y, inter_x, inter_y, x[i], y[i])
            if angle < minAngle:
                minAngle = angle
                minAngleIndex = j
                end_inter_x = inter_x; end_inter_y = inter_y
        
        start_inter_x = end_inter_x
        start_inter_y = end_inter_y

    i = minAngleIndex
    traj_x.append(end_inter_x)
    traj_y.append(end_inter_y)

    # now we are sure that first point is not in unallowed position and that it is the right intersection
    # repeat algorithm and add points to trajectory
    while minAngleIndex >= i:
        i = minAngleIndex
        minAngle = 3*pi     # inf
        
        cnt = 0; j = i; check_pts = len(x)/4
        end_inter_x = 0; end_inter_y = 0
        while cnt < check_pts:
            # j = j+1;
            j = (j+1) % len(x)
            cnt = cnt+1
            
            if sqrt( (x[i]-x[j])**2 + (y[i]-y[j])**2 ) > 2*dist:
                continue
            
            [inter_x, inter_y] = intersection_ccw(x[i], y[i], x[j], y[j], dist, dist, start_inter_x, start_inter_y)
            angle = getAngle_ccw(start_inter_x, start_inter_y, inter_x, inter_y, x[i], y[i])
            if angle < minAngle:
                minAngle = angle
                minAngleIndex = j
                end_inter_x = inter_x; end_inter_y = inter_y
        
        # trajectory
        new_traj_points_x, new_traj_points_y, s = \
            make_traj_points(start_inter_x, start_inter_y, end_inter_x, end_inter_y, x[i], y[i], s, res, dist)

        traj_x += new_traj_points_x
        traj_y += new_traj_points_y

        start_inter_x = end_inter_x
        start_inter_y = end_inter_y


    traj_yaw = []
    end = len(traj_x)-1
    for i in range(len(traj_x)):
        if i == 0:
            alpha = getAngle_ccw(traj_x[i+1], traj_y[i+1], traj_x[end], traj_y[end], traj_x[i], traj_y[i]) / 2
            alpha = atan2(traj_y[i+1]-traj_y[i], traj_x[i+1]-traj_x[i]) + alpha
        elif i == end:
            alpha = getAngle_ccw(traj_x[0], traj_y[0], traj_x[i-1], traj_y[i-1], traj_x[i], traj_y[i]) / 2
            alpha = atan2(traj_y[0]-traj_y[end], traj_x[0]-traj_x[end]) + alpha
        else:
            alpha = getAngle_ccw(traj_x[i+1], traj_y[i+1], traj_x[i-1], traj_y[i-1], traj_x[i], traj_y[i]) / 2
            alpha = atan2(traj_y[i+1]-traj_y[i], traj_x[i+1]-traj_x[i]) + alpha
        
        if alpha > pi:
            alpha = alpha - 2*pi

        traj_yaw.append(alpha)

    h = height
    traj_z = [h] * len(traj_x)
    tmp_traj_x = traj_x[:]
    tmp_traj_y = traj_y[:]
    tmp_traj_yaw = traj_yaw[:]
    while h-res_height >= MIN_HEIGHT:
        h -= res_height
        traj_x += tmp_traj_x
        traj_y += tmp_traj_y
        traj_yaw += tmp_traj_yaw
        traj_z += [h] * len(tmp_traj_x)

    return traj_x, traj_y, traj_z, traj_yaw


if __name__ == "__main__":
    rospy.init_node("generate_trajectory_points_service_node")
    GenerateTrajectory()
    rospy.spin()
