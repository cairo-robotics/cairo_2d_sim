import json
import os
import copy
import uuid

import rospy
from std_msgs.msg import Bool

from cairo_2d_sim.msg import MenuCommands, Pose2DStamped

CURRENT_WORKING_DIRECTORY = os.getcwd()

class Record:
    
    def __init__(self):
        self.demonstration = []
        self.robot_state_sub = rospy.Subscriber('/cairo_2d_sim/robot_state', Pose2DStamped, self.robot_state_cb)
        self.constraint_one = rospy.Subscriber('/cairo_2d_sim/constraint_one', Bool, self.constraint_one_cb)
        self.constraint_two = rospy.Subscriber('/cairo_2d_sim/constraint_two', Bool, self.constraint_two_cb)
        self.constraint_three = rospy.Subscriber('/cairo_2d_sim/constraint_three', Bool, self.constraint_three_cb)
        self.menu_commands = rospy.Subscriber('/cairo_2d_sim/menu_commands', MenuCommands, self._menu_commands_cb)
        self.curr_robot_state = {}
        self.curr_constraints = {
            'c1': False,
            'c2': False,
            'c3': False
        }
        self.record_state = {
            "capture": False,
            "restart": False,
            "quit": False
        }
    
    def robot_state_cb(self, msg):
        self.curr_robot_state['time'] = msg.header.stamp.to_sec()
        self.curr_robot_state['x'] = msg.pose2d.x
        self.curr_robot_state['y'] = msg.pose2d.y
        self.curr_robot_state['theta'] = msg.pose2d.theta
        
    def constraint_one_cb(self, msg):
        self.curr_constraints['c1'] = msg.data
    
    def constraint_two_cb(self, msg):
        self.curr_constraints['c2'] = msg.data
        
    def constraint_three_cb(self, msg):
        self.curr_constraints['c3'] = msg.data
        
    def record(self):
        while True:
            self.observe()
            if self.record_state['restart']:
                self.demonstration = []
            if self.record_state['capture']:
                self.save_demonstration()
                self.record_state['capture'] = False
                rospy.sleep(0.5)
            if self.record_state['quit']:
                rospy.signal_shutdown("Quit signal received.")
                break
            rospy.sleep(0.01)
            # Going any faster than this rate will cause the program to save demonstrations repeatedly.
    
    def observe(self):
        observation = {}
        observation['robot_state'] = copy.deepcopy(self.curr_robot_state)
        observation['constraints'] = copy.deepcopy(self.curr_constraints)
        self.demonstration.append(observation)
    
    def save_demonstration(self):
        filename = str(uuid.uuid4())
        file_path = os.path.join(CURRENT_WORKING_DIRECTORY, filename + '_demo.json')
        print("Saving demonstration...")
        with open(file_path, 'w') as f:
            json.dump(self.demonstration, f)
        self.demonstration = []
        
    def _menu_commands_cb(self, msg):
        self.record_state["quit"] = msg.quit.data
        self.record_state["capture"] = msg.capture.data
        self.record_state["restart"] = msg.restart.data
