import json
import os

import rospy
from geometry_msgs.msg import Pose2D

from cairo_2d_sim.msg import ConstraintToggles, KeyboardArrows, MousePress

CURRENT_WORKING_DIRECTORY = os.getcwd()

class Record:
    
    def __init__(self):
        self.demonstration = []
        self.robot_state_sub = rospy.Subscriber('/cairo_2d_sim/robot_state', Pose2D, self.robot_state_cb)
        self.constraint_toggles_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, self.constraint_cb)
        rospy.on_shutdown(self.save_demonstration)
        self.curr_robot_state = {}
        self.curr_constraints = {}
    
    def robot_state_cb(self, msg):
        self.curr_robot_state['x'] = msg.x
        self.curr_robot_state['y'] = msg.y
        self.curr_robot_state['theta'] = msg.theta
        
    def constraint_cb(self, msg):
        self.curr_constraints['c1'] = msg.c1.data
        self.curr_constraints['c2'] = msg.c2.data
        self.curr_constraints['c3'] = msg.c3.data
    
    def capture_demonstration(self):
        observation = {}
        observation['robot_state'] = self.curr_robot_state
        observation['constraints'] = self.curr_constraints
        self.demonstration.append(observation)
    
    def save_demonstration(self):
        file_path = os.path.join(CURRENT_WORKING_DIRECTORY, 'demonstration.json')
        print("Saving demonstration...")
        print(self.demonstration)
        with open(file_path, 'w') as f:
            json.dump(self.demonstration, f)