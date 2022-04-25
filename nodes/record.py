#! /usr/bin/env python3
import rospy

from cairo_2d_lfd.record import Record
     
if __name__ == '__main__':
    recorder = Record()
    rospy.init_node('demonstration_recorder')
    rospy.spin()

