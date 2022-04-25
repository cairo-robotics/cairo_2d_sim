import os

import pygame as pg

IMAGE_FILE_DIR = os.path.dirname(os.path.realpath(__file__)) + "/../../../data/img/"

def draw_rect_alpha(screen, color, rect):
    shape_surf = pg.Surface(pg.Rect(rect).size, pg.SRCALPHA)
    pg.draw.rect(shape_surf, color, shape_surf.get_rect())
    screen.blit(shape_surf, rect)
    
def draw_circle_alpha(screen, color, center, radius):
    target_rect = pg.Rect(center, (0, 0)).inflate((radius * 2, radius * 2))
    shape_surf = pg.Surface(target_rect.size, pg.SRCALPHA)
    pg.draw.circle(shape_surf, color, (radius, radius), radius)
    screen.blit(shape_surf, target_rect)

def text_objects(text, font):
    txtsurf = font.render(text, True, (0, 0, 0))
    return txtsurf, txtsurf.get_rect()


def disptf(x, y, theta=0):
    a = [(x+500), (380-y), -theta]
    return a

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
   
