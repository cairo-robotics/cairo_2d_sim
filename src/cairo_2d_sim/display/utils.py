import os
import math
import json

import pygame as pg

IMAGE_FILE_DIR = os.path.dirname(
    os.path.realpath(__file__)) + "/../../../data/img/"


def draw_rect_alpha(screen, color, rect):
    shape_surf = pg.Surface(pg.Rect(rect).size, pg.SRCALPHA)
    pg.draw.rect(shape_surf, color, shape_surf.get_rect())
    screen.blit(shape_surf, rect)


def draw_circle_alpha(screen, color, center, radius):
    target_rect = pg.Rect(center, (0, 0)).inflate((radius * 2, radius * 2))
    shape_surf = pg.Surface(target_rect.size, pg.SRCALPHA)
    pg.draw.circle(shape_surf, color, (radius, radius), radius)
    screen.blit(shape_surf, target_rect)


def draw_arrow_alpha(screen, color, pos, angle):
    angle = angle + 90
    body_length = 20
    width = 15
    # Create the triangle head around the origin
    head_verts = [
        pg.Vector2(0, width / 2),  # Center
        pg.Vector2(width / 2, -width / 2),  # Bottomright
        pg.Vector2(-width / 2, -width / 2),  # Bottomleft
    ]
    # Rotate and translate the head into place
    translation = pg.Vector2(0, body_length - (width / 2)).rotate(-angle)
    for i in range(len(head_verts)):
        head_verts[i].rotate_ip(-angle)
        head_verts[i] += translation
        head_verts[i] += pos

    pg.draw.polygon(screen, color, head_verts)


def text_objects(text, font):
    txtsurf = font.render(text, True, (0, 0, 0))
    return txtsurf, txtsurf.get_rect()

def disptf(x, y, theta=0):
    a = [(x+500), (380-y), -theta]
    return a

def rotate(pos, angle):
    cen = (5+pos[0], 0+pos[1])
    angle *= -(math.pi/180)
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    ret = ((cos_theta * (pos[0] - cen[0]) - sin_theta * (pos[1] - cen[1])) + cen[0],
           (sin_theta * (pos[0] - cen[0]) + cos_theta * (pos[1] - cen[1])) + cen[1])
    return ret


def blit_rotate(image, target_pos, original_pos, angle):
    # offset from pivot to center
    image_rect = image.get_rect(
        topleft=(target_pos[0] - original_pos[0], target_pos[1] - original_pos[1]))
    offset_center_to_pivot = pg.math.Vector2(target_pos) - image_rect.center

    # roatated offset from pivot to center
    rotated_offset = offset_center_to_pivot.rotate(-angle)

    # roatetd image center
    rotated_image_center = (
        target_pos[0] - rotated_offset.x, target_pos[1] - rotated_offset.y)

    # get a rotated image
    rotated_image = pg.transform.rotate(image, angle)
    rotated_image_rect = rotated_image.get_rect(
        center=rotated_image_center)

    return rotated_image, rotated_image_rect
