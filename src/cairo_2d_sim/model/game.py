
import sys, signal
import json

import pygame as pg
import rospy
from std_msgs.msg import String

from cairo_2d_sim.model.statics import DirectionalCircleStatic, LineStatic, TextLabelStatic
from cairo_2d_sim.display.display import Display
from cairo_2d_sim.msg import MenuCommands



def signal_handler(signal, frame):
  pg.quit()
  sys.exit(0)


class Game():
    
    def __init__(self, screen, environment, sprites, statics, controllers, toggles):
        self.screen = screen
        self.display = Display(environment, sprites, statics, toggles)
        self.sprites = sprites
        self.statics = statics
        self.controllers = controllers
        self.toggles = toggles
        self.menu_commands_state = {
            "quit": False,
            "restart": False,
            "capture": False
        }
        self.menu_commands = rospy.Subscriber('/cairo_2d_sim/menu_commands', MenuCommands, self._menu_commands_cb)
        
    def _update_toggles(self):
        for toggle in self.toggles:
            toggle.update()
        
    def _update_controls(self, event):
        for controller in self.controllers:
            controller.update(event)
    
    def _update_sprites(self):
        for sprite in self.sprites:
            sprite.update()
            
    def _render(self):
        self.display.render(self.screen)
    
    def check_quit(self):
        return self.menu_commands_state['quit']
    
    def check_restart(self):
        return self.menu_commands_state['restart']

    def check_capture(self):
        return self.menu_commands_state['capture']
    
    def step(self):
        for event in pg.event.get():
            self._update_controls(event)
        self._update_toggles()
        self._update_sprites()
        self._render()
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
    
    def _menu_commands_cb(self, msg):
        self.menu_commands_state["quit"] = msg.quit.data
        self.menu_commands_state["capture"] = msg.capture.data
        self.menu_commands_state["restart"] = msg.restart.data

class Replay():
    
    def __init__(self, screen, environment, sprites, statics, controllers, toggles):
        self.screen = screen
        self.display = Display(environment, sprites, statics, toggles)
        self.sprites = sprites
        self.statics = statics
        self.controllers = controllers
        self.toggles = toggles
        self.menu_commands_state = {
            "quit": False,
            "restart": False,
            "capture": False
        }
        self.menu_commands = rospy.Subscriber('/cairo_2d_sim/menu_commands', MenuCommands, self._menu_commands_cb)
        self.circle_static_sub = rospy.Subscriber('/cairo_2d_sim/create_directional_circle_static', String, self._create_directional_circle_static_cb)
        self.line_static_sub = rospy.Subscriber('/cairo_2d_sim/create_line_static', String, self._create_line_static_cb)
        self.text_label_sub = rospy.Subscriber('/cairo_2d_sim/create_text_label', String, self._create_text_label_cb)
    
    def check_quit(self):
        return self.menu_commands_state['quit']
    
    def check_restart(self):
        return self.menu_commands_state['restart']

    def check_capture(self):
        return self.menu_commands_state['capture']
    
    def step(self):
        self._update_toggles()
        self._update_sprites()
        self._render()
        signal.signal(signal.SIGTERM, signal_handler)
        signal.signal(signal.SIGINT, signal_handler)
    
    def _menu_commands_cb(self, msg):
        self.menu_commands_state["quit"] = msg.quit.data
        self.menu_commands_state["capture"] = msg.capture.data
        self.menu_commands_state["restart"] = msg.restart.data
        
    def _update_toggles(self):
        for toggle in self.toggles:
            toggle.update()
            
    def _update_sprites(self):
        for sprite in self.sprites:
            sprite.replay()
            
    def _render(self):
        self.display.render(self.screen)
        
    def _create_directional_circle_static_cb(self, msg):
        d = json.loads(msg.data)
        new_static = DirectionalCircleStatic(d['x'], d['y'], d['angle'], d['radius'], d['color'])
        self.statics.append(new_static)
        self.display.update_statics(self.statics)
    
    def _create_line_static_cb(self, msg):
        d = json.loads(msg.data)
        new_static = LineStatic(d['x1'], d['y1'], d['x2'], d['y2'], d['color'])
        self.statics.append(new_static)
        self.display.update_statics(self.statics)
    
    def _create_text_label_cb(self, msg):
        d = json.loads(msg.data)
        new_static = TextLabelStatic(d['x'], d['y'], d['text'], d['color'])
        self.statics.append(new_static)
        self.display.update_statics(self.statics)

        
    
 
        
    
    