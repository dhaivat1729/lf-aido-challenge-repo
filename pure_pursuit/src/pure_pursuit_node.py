#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import Segment, SegmentList, Twist2DStamped

def callback(data):

    segment_list = data.segments

    ## White color is 0 and yellow color is 1
    ## let's calculate which one are higher
    white = 0
    yellow = 0
    lookup_dist = 0.3
    near_by_points_x_white = []
    near_by_points_y_white = []
    near_by_points_x_yellow = []
    near_by_points_y_yellow = []
    dist_thresh = 2e-1

    for i in range(len(segment_list)):
        if segment_list[i].color == 0:
            white += 1
            if abs(segment_list[i].points[0].x - lookup_dist) < dist_thresh:
                near_by_points_x_white.append(segment_list[i].points[0].x)
                near_by_points_y_white.append(segment_list[i].points[0].y)
            if abs(segment_list[i].points[1].x - lookup_dist) < dist_thresh:
                near_by_points_x_white.append(segment_list[i].points[1].x)
                near_by_points_y_white.append(segment_list[i].points[1].y)
        elif segment_list[i].color == 1:
            yellow += 1
            # print(segment_list[i].points[0].x)
            if abs(segment_list[i].points[0].x - lookup_dist) < dist_thresh:
                near_by_points_x_yellow.append(segment_list[i].points[0].x)
                near_by_points_y_yellow.append(segment_list[i].points[0].y)
            if abs(segment_list[i].points[1].x - lookup_dist) < dist_thresh:
                near_by_points_x_yellow.append(segment_list[i].points[1].x)
                near_by_points_y_yellow.append(segment_list[i].points[1].y)

    vel_msg = Twist2DStamped()

    # import pdb; pdb.set_trace()
    # print(vel_msg)
    x_follow, y_follow = None, None
    if len(near_by_points_x_white) > 5:
        x_follow = np.mean(np.array(near_by_points_x_white))
        y_follow = np.mean(np.array(near_by_points_y_white)) + 0.15
    if len(near_by_points_x_yellow) > 5 and (x_follow == None and y_follow == None):
        x_follow = np.mean(np.array(near_by_points_x_yellow))
        y_follow = np.mean(np.array(near_by_points_y_yellow)) - 0.15    

    print(x_follow, y_follow)

    v = None

    if len(near_by_points_x_white) != 0:
        x_white_var = np.var(np.array(near_by_points_x_white))
        y_white_var = np.var(np.array(near_by_points_y_white))
        if y_white_var < x_white_var:
            v = 0.8
            mul_const = 4
        else:
            v = 0.2
            mul_const = 9

    if v == None and len(near_by_points_x_yellow) > 0:
        x_yellow_var = np.var(np.array(near_by_points_x_yellow))
        y_yellow_var = np.var(np.array(near_by_points_y_yellow))
        if y_yellow_var < x_yellow_var:
            v = 0.8
            mul_const = 4
        else:
            v = 0.2      
            mul_const = 9  

    ## if there are no detections
    if x_follow == None and y_follow == None:
        ## publish some small omega because we can't keep moving
        vel_msg.v = 0.0
        vel_msg.omega = 0.4
        pub.publish(vel_msg)    

    else:
        alpha = np.arctan2(y_follow, x_follow)
        vel_msg.v = v
        vel_msg.omega = mul_const*vel_msg.v*(np.sin(alpha))/lookup_dist 
        print(vel_msg.v, vel_msg.omega)

        pub.publish(vel_msg)

    
def listener():

    
    

    rospy.Subscriber("~seglist_filtered", SegmentList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=10)
    listener()