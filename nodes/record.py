#! /usr/bin/env python3
import rospy

from cairo_2d_sim.control.record import Record
     
if __name__ == '__main__':
    recorder = Record()
    rospy.init_node('demonstration_recorder')
    while not rospy.is_shutdown():
        recorder.record()
        rospy.signal_shutdown("Quit signal received.")

