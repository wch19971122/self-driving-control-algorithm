#!/usr/bin/env python3
import sys
import os
from cmath import sqrt
from importlib import import_module
from os import path
from weakref import ref
import rospy
import numpy as np
import scipy.io as scio 
import pandas as pd
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from  pure_persuit.msg import referencelinePoint
from pure_persuit.msg import referenceline


def referenceline_generate() :

    ref_line = referenceline();
    data = scio.loadmat("/home/wuchaohui/catkin_ws/src/LQR/scripts/path.mat")
    xr = np.array(data['global_path_x'])
    yr = np.array(data['global_path_y'])
    xr = xr.reshape(xr.shape[0],)
    yr = yr.reshape(yr.shape[0],)
    

    # x = np.arange(0, 101, 25)
    # y = [0, 10, 25, 40, 50]
    # an = np.polyfit(x, y, 3)
    # xr = np.arange(0, 100, 0.5)
    # yr = np.polyval(an, xr)
    
    dx = np.diff(xr)
    dy = np.diff(yr)

    dxfront = np.append(dx[0],dx)
    dxrear = np.append(dx,dx[-1])

    dxfinal = (dxfront+dxrear)/2

    dyfront = np.append(dy[0],dy)
    dyrear = np.append(dy,dy[-1])

    dyfinal = (dyfront+dyrear)/2

    dsfinal =np.sqrt(np.power(dxfinal,2)+np.power(dyfinal,2))

    path_heading = np.arctan2(dyfinal,dxfinal)
    dheading = np.diff(path_heading)
    dheadingfront = np.append(dheading[0],dheading)
    dheadingrear = np.append(dheading,dheading[-1])
    dheadingfinal = (dheadingfront+dheadingrear)/2

    path_kappa = np.sin(dheadingfinal)/dsfinal

    for i in range(len(xr)):
        ref_point = referencelinePoint(xr = xr[i], yr = yr[i], thetar = path_heading[i], kr = path_kappa[i])
        ref_line.referencelinePoints.append(ref_point)

    return ref_line 


if __name__ ==  '__main__':
    ref_line = referenceline_generate()
    rospy.init_node('referenceline_publisher',anonymous = True)
    rate = rospy.Rate(10)

    ref_pub = rospy.Publisher('/referenceline_pub', referenceline,queue_size =1,latch = True)

    center_pub = rospy.Publisher('/lqr_centerline_pub', Path, queue_size = 1,latch = True)
    msg = Path()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    for pt in ref_line.referencelinePoints:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = pt.xr
        pose.pose.position.y = pt.yr
        msg.poses.append(pose)
    
    while not rospy.is_shutdown():
        ref_pub.publish(ref_line)
        center_pub.publish(msg)
        rate.sleep()

    rospy.spin()







