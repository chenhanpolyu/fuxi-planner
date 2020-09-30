#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 22 10:18:26 2020

@author: chen
"""
import rospy
from mavros_test_common import MavrosTestCommon
if __name__ == "__main__":
    rospy.init_node('UAV_controller')
    uavmode=MavrosTestCommon()
    uavmode.setUp()
    uavmode.set_arm(True, 5)  
    try:
        uavmode.set_mode("OFFBOARD", 20)
    except:
        print("set mode fail")