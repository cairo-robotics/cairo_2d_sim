from cairo_2d_sim.control.input import MousePositionInput, MousePressInput, KeyboardArrowsInput, ConstraintTogglesInput

class HolonomicController():
    
    def __init__(self):
        self.mouse_position = MousePositionInput()
        self.mouse_press = MousePressInput()
        self.keyboard = KeyboardArrowsInput()
    
    def update(self):
        self.mouse_position.update()
        self.mouse_press.update()
        self.keyboard.update()
        
class InterfaceController():
    
    def __init__(self):
        self.constraints = ConstraintTogglesInput()
    
    def update(self):
        self.constraints.update()
