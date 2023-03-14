import json

def publish_directed_point(publisher, position, angle, radius, color):
    data = {}
    data["x"] = position[0]
    data["y"] = position[1]
    data["radius"] = radius
    data["angle"] = angle
    data["color"] = color
    data_str = json.dumps(data)
    publisher.publish(data_str)

def publish_line(publisher, pos1, pos2, color):
    data = {}
    data["x1"] = pos1[0]
    data["y1"] = pos1[1]
    data["x2"] = pos2[0]
    data["y2"] = pos2[1]
    data["color"] = color
    data_str = json.dumps(data)
    publisher.publish(data_str)

def publish_text_label(publisher, pos, text, color):
    data = {}
    data["x"] = pos[0]
    data["y"] = pos[1]
    data["text"] = text
    data["color"] = color
    data_str = json.dumps(data)
    publisher.publish(data_str)