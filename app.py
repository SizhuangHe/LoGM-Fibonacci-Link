import pymunk
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d

import pygame
from pygame.locals import *

import math
from PIL import Image

pymunk.pygame_util.positive_y_is_up = False # The +x direction is to the right of the screen and +y is to the bottom of the screen
space = pymunk.Space()
space.gravity = (0, 900) # This defines a gravity pointing downwards
space.damping = 0.7 # This parameter defines the speed energy is lost during motion. This is defaulted to 1, meaning no energy loss. 
                    # For example, 0.7 here means 30% of energy is lost every second. 
                    # Setting it to a number less than 1 makes moving objects to finally settle down.

b0 = space.static_body
size = w, h = 800, 800
fps = 100   # This parameter defines how "fast" time goes by. With larger fps, time goes "slowly".
steps = 10

BLACK = (0, 0, 0)
GRAY = (230, 230, 230)
WHITE = (255, 255, 255)


# The following classes define physical objects that we may use in the simulation
class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)


class PivotJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)

class DampedRotarySpring:
    def __init__(self, b, b2, angle, stiffness, damping):
        joint = pymunk.constraints.DampedRotarySpring(
            b, b2, angle, stiffness, damping)
        space.add(joint)


class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)

class Segment:
    def __init__(self, p0, v, radius=4):
        self.body = pymunk.Body()
        self.body.position = p0
        shape = pymunk.Segment(self.body, (0, 0), v, radius)
        shape.density = 0.1
        shape.elasticity = 0.5
        shape.filter = pymunk.ShapeFilter(group=1)
        shape.color = (47, 141, 255,0)
        space.add(self.body, shape)


class App:
    
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(size)
        self.draw_options = DrawOptions(self.screen)
        self.running = True
        self.gif = 0
        self.images = []
        self.selected = None
        self.mouse_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)

    def run(self):
        while self.running:
            for event in pygame.event.get():
                self.do_event(event)

            self.draw()
            self.clock.tick(fps)

            for i in range(steps):
                space.step(1/fps/steps)
        
        pygame.quit()

    def do_event(self, event):
        mouse_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        selected = None
        if event.type == pygame.QUIT:
                self.running = False

        if event.type == pygame.MOUSEBUTTONDOWN: # user clicked the mouse
                if self.selected != None:
                    space.remove(self.selected)
                p = from_pygame(Vec2d(*event.pos))
                hit = space.point_query_nearest(p, 0, pymunk.ShapeFilter())
                if hit != None:
                    shape = hit.shape
                    rest_length = self.mouse_body.position.get_distance(shape.body.position)
                    ds = pymunk.DampedSpring(
                        self.mouse_body, shape.body, (0, 0), (0, 0), rest_length, 1000, 1
                    ) # put a damped string between the cursor to the body on the chain, simulation a pulling force
                    space.add(ds)
                    self.selected = ds
        elif event.type == pygame.MOUSEBUTTONUP:
                if self.selected != None:
                    space.remove(self.selected)
                    self.selected = None
        
        mpos = pygame.mouse.get_pos()
        p = from_pygame(Vec2d(*mpos))
        self.mouse_body.position = p


    def draw(self):
        self.screen.fill(WHITE)
        space.debug_draw(self.draw_options)
        pygame.display.update()

        text = f'fpg: {self.clock.get_fps():.1f}'
        pygame.display.set_caption(text)
        self.make_gif()

    def make_gif(self):
        if self.gif > 0:
            strFormat = 'RGBA'
            raw_str = pygame.image.tostring(self.screen, strFormat, False)
            image = Image.frombytes(
                strFormat, self.screen.get_size(), raw_str)
            self.images.append(image)
            self.gif -= 1
            if self.gif == 0:
                self.images[0].save('joint.gif',
                                    save_all=True, append_images=self.images[1:],
                                    optimize=True, duration=1000//fps, loop=0)
                self.images = []


def to_pygame(p):
    # convert pymunk to pygame coordinates"""
        return int(p.x), int(-p.y + 800)

def from_pygame(p):
    return to_pygame(p)

def lucas_array_generator(length):
    if length < 2:
        print("less than 2 links!")
        return
    lucas_array = [2, 1]
    for i in range(2, length):
        ele = lucas_array[i - 2] + lucas_array[i - 1]
        lucas_array.append(ele)  
    return lucas_array    

def mirror_wrt_y_axis(arr):
    """
    Inputs:
        -- arr: an array of Vec2d
    Outputs:
        --mirror_arr: an array of Vec2d where each element is a Vec2d with x-coordinate flipped    
    """
    mirror_arr = []

    for item in arr:
        mirror_arr.append(Vec2d(-item.x, item.y))

    return mirror_arr

def generate_half_link(hanging_pt, v_arr):
    """
    This function generates half of a discrete chain.
    Inputs:
        -- hanging_pt: Vec2d. This is the point where this half of the chain is hanging.
        -- v_arr: an array of Vec2d. This stores the slopes of each link
    Outputs:
        -- previous_body: body object. This is the body of the lowest segment. Return this to be connected to the other half.    
    """
    
    top_segment_1 = Segment(hanging_pt, v_arr[0])
    PivotJoint(b0, top_segment_1.body, hanging_pt)
    top_segment_2 = Segment(hanging_pt + v_arr[0], v_arr[0])
    PivotJoint(top_segment_1.body, top_segment_2.body, v_arr[0])
    previous_body = top_segment_2.body

    for i in range(1, len(v_arr)):
        pivot_point = hanging_pt
        for j in range(i):
            pivot_point += 2 * v_arr[j]
        seg_1 = Segment(pivot_point, v_arr[i])
        RotaryLimitJoint(previous_body, seg_1.body, 0, 0)
        PinJoint(previous_body, seg_1.body, v_arr[i-1]) #v[i]?
        seg_2 = Segment(pivot_point+v_arr[i], v_arr[i])
        PivotJoint(seg_1.body, seg_2.body, v_arr[i])
        previous_body = seg_2.body
    
    return previous_body    

def connect_two_halves(lower_body_1, lower_body_2, v_arr_1, v_arr_2):
    RotaryLimitJoint(lower_body_2, lower_body_1, 0, 0)
    PinJoint(lower_body_2, lower_body_1, v_arr_2[len(v_arr_2)-1], v_arr_1[len(v_arr_1)-1])

def generate_link(num_links):
    lucas = lucas_array_generator(num_links * 2 + 1)
    
    v = [] # the slope of each link
    # define the unit length and the position of the top two pivot points to make the plot fit the window
    unit_length = 700/(lucas[len(lucas)-1])
    hanging_pt1 = Vec2d(400 - 2* num_links * unit_length,0)
    hanging_pt2 = Vec2d(400 + 2* num_links * unit_length,0)

    for i in range(num_links):
        v.append(Vec2d(-1, 0.5*(lucas[2*(i+1)]-lucas[2*i]))*unit_length)
    
    v.reverse() # reverse the list v because we are building the links from top which corresponds to the backwards direction of v
    v_mirror = mirror_wrt_y_axis(v)

    # generate the right half and the left half of the discrete link
    lower_body_1 = generate_half_link(hanging_pt=hanging_pt2, v_arr=v)
    lower_body_2 = generate_half_link(hanging_pt=hanging_pt1, v_arr=v_mirror)
    #connect the two halves at the lowest point
    connect_two_halves(lower_body_1, lower_body_2, v, v_mirror)



if __name__ == '__main__':
    generate_link(4)
    App().run()