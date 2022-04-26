#! /usr/bin/env python3
import rospy

from cairo_2d_sim.lfd.record import Record
     
if __name__ == '__main__':
    recorder = Record()
    rospy.init_node('demonstration_recorder')
    while not rospy.is_shutdown():
        recorder.capture_demonstration()
        rospy.sleep(0.1)

