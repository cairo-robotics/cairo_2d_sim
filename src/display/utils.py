

def text_objects(text, font):
    txtsurf = font.render(text, True, (0, 0, 0))
    return txtsurf, txtsurf.get_rect()


def disptf(x, y, theta=0):
    a = [(x+500), (380-y), -theta]
    return a

