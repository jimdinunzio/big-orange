import pygame
from pygame.locals import *
from pygame.compat import geterror
import math
import random

WIDTH = 1024
HEIGHT = 600

_going = False
_angle = 0
_offset = 0
_dims = None

def update():
    pass

def draw_eyes(screen, pupil_angle=0, pupil_offset=0):
#    screen.fill((0, 0, 0))
    pupil_angle = math.radians(pupil_angle)
    def draw_eye(eye_x, eye_y, pupil_angle, pupil_offset):
        #mouse_x, mouse_y = pygame.mouse.get_pos()

        #distance_x = mouse_x - eye_x
        #distance_y = mouse_y - eye_y
        #distance = min(math.sqrt(distance_x**2 + distance_y**2), 70)
        pupil_offset = min(pupil_offset, 70)
        #angle = math.atan2(distance_y, distance_x)

        pupil_x = int(eye_x + (math.cos(pupil_angle) * pupil_offset))
        pupil_y = int(eye_y + (math.sin(pupil_angle) * pupil_offset))

        pygame.draw.circle(screen, (255, 255, 255), (eye_x, eye_y), 150)
        pygame.draw.circle(screen, (0, 0, 100), (pupil_x, pupil_y), 50)

    draw_eye(_dims[0] / 3 - 40, _dims[1]/ 2, pupil_angle, pupil_offset)
    draw_eye(2*_dims[0] / 3 + 40, _dims[1]/ 2, pupil_angle, pupil_offset)

def shutdown():
    global _going
    _going = False

def set(angle, offset):
    global _angle, _offset
    _angle = angle
    _offset = offset

def next_blink_time():
    return pygame.time.get_ticks() + 1000 + (random.random()*6000)

def start():
    global _going, _dims
    _going = True

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    _dims = screen.get_size()
#    pygame.display.set_caption('Orange Eyes')

       # Create The Backgound
    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((0, 0, 0))

      # Put Text On The Background, Centered
    # if pygame.font:
    #     font = pygame.font.Font(None, 36)
    #     text = font.render("Orange Eyes", 1, (10, 10, 10))
    #     textpos = text.get_rect(centerx=background.get_width()/2)
    #     background.blit(text, textpos)

    # Display The Background
    screen.blit(background, (0, 0))
    pygame.display.flip()

     # Prepare Game Objects
    clock = pygame.time.Clock()
    time = pygame.time
    time_to_blink = next_blink_time()

    # Main Loop
    try:
        while _going:
            clock.tick(30)

            # Handle Input Events
            # for event in pygame.event.get():
            #     if event.type == QUIT:
            #         going = False
            #     elif event.type == KEYDOWN and event.key == K_ESCAPE:
            #         going = False

            #allsprites.update()

            # Draw Everything
            screen.blit(background, (0, 0))
            #allsprites.draw(screen)
            draw_eyes(screen, _angle, _offset)

            #blink
            if time.get_ticks() > time_to_blink:
                pygame.draw.rect(screen, (0,0,0), (0, 0, _dims[0], _dims[1]/2))
                pygame.display.flip()
                time.wait(40)
                pygame.draw.rect(screen, (0,0,0), (0, _dims[1]/2, _dims[0], _dims[1] / 2))
                pygame.display.flip()
                time.wait(300)
                time_to_blink = next_blink_time()

            pygame.display.flip()
    except KeyboardInterrupt:
        None

    pygame.quit()
