from cairo_2d_sim.control.input import MousePositionInput, MousePressInput, KeyboardArrowsInput, ConstraintTogglesInput

class HolonomicController():
    
    def __init__(self):
        self.mouse_position = MousePositionInput()
        self.mouse_press = MousePressInput()
        self.keyboard = KeyboardArrowsInput()
        self.constraints = ConstraintTogglesInput()
    
    def update(self):
        self.mouse_position.update()
        self.mouse_press.update()
        self.keyboard.update()
