from cairo_2d_sim.control.input import MousePositionInput, MousePressInput, KeyboardArrowsInput, ConstraintTogglesInput

class HolonomicController():
    
    def __init__(self):
        self.mouse_position = MousePositionInput()
        self.mouse_press = MousePressInput()
        self.keyboard = KeyboardArrowsInput()
    
    def update(self, event):
        self.mouse_position.update(event)
        self.mouse_press.update(event)
        self.keyboard.update(event)
        
class InterfaceController():
    
    def __init__(self):
        self.constraint_toggles = ConstraintTogglesInput()
    
    def update(self, event):
        self.constraint_toggles.update(event)
