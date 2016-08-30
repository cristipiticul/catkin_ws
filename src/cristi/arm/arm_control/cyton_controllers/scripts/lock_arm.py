#!/usr/bin/env python

import sys
import os

import rospy

from dynamixel_controllers.srv import *

services = [
  '/shoulder_roll_position_controller/torque_enable',
  '/shoulder_pitch_position_controller/torque_enable',
  '/shoulder_yaw_position_controller/torque_enable',
  '/elbow_pitch_position_controller/torque_enable',
  '/elbow_yaw_position_controller/torque_enable',
  '/wrist_pitch_position_controller/torque_enable',
  '/wrist_roll_position_controller/torque_enable',
  '/gripper_controller/torque_enable'
]
nr_of_services = 8
def call_service(service_name, parameter):

    rospy.wait_for_service(service_name)
    
    try:
        serv = rospy.ServiceProxy(service_name, TorqueEnable)
        print "Calling service %s with parameter %d" % (service_name, parameter.torque_enable)
        resp = serv.call(parameter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    ok = True
    argv = rospy.myargv()
    if argv[1] == 'lock':
      torque_enable = True
    elif argv[1] == 'unlock':
      torque_enable = False
    else:
      print "Unknown parameter %s. It must be either 'lock' or 'unlock'" % argv[1]
      ok = False
    if ok:
      for i in range(0, nr_of_services):
        parameter = TorqueEnableRequest()
        parameter.torque_enable = torque_enable
        call_service(services[i], parameter)
