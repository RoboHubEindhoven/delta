#!/usr/bin/env python

import time
import rospy
from DRV90L import DRV90L
from end_effector import EndEffector

if __name__ == "__main__":
    s = DRV90L()
    e = EndEffector()
    #rospy.spin()
    Home = s.getSavedToolPose("Home")
    p1 = s.getSavedToolPose("p1")
    p2 = s.getSavedToolPose("p2")
    p3 = s.getSavedToolPose("p3")
    p4 = s.getSavedToolPose("p4")
    END = s.getSavedToolPose("END")
    while not rospy.is_shutdown():
        print("Starting loop")
        e.stopDispensing()
        time.sleep(1)
        s.sendPositionMove(Home[0], Home[1], Home[2], Home[3], Home[4], Home[5], 50, 'world')
        e.stopBrushing()
        time.sleep(1)
        e.startBrushing()
        s.sendPositionMove(p1[0], p1[1], p1[2], p1[3], p1[4], p1[5], 50, 'world')
        e.startDispensing()
        s.sendArcMove([p2[0], p2[1], p2[2], p2[3], p2[4], p2[5]], [p3[0], p3[1], p3[2], p3[3], p3[4], p3[5]])
        time.sleep(1)
        s.sendArcMove([p4[0], p4[1], p4[2], p4[3], p4[4], p4[5]], [p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]])
        time.sleep(1)
        e.stopBrushing()
        e.stopDispensing()
        s.sendPositionMove(Home[0], Home[1], Home[2], Home[3], Home[4], Home[5], 50, 'world')
        time.sleep(1)
        e.startBrushing()
        e.startDispensing()
        s.sendPositionMove(END[0], END[1], END[2], END[3], END[4], END[5], 50, 'world')