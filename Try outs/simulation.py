import pymunk
import sys
import pygame
import pymunk.pygame_util

def add_tuple(t1,t2):
    return (t1[0]+t2[0],t1[1]-t2[1])

def to_pygame(p):
    """Small helper to convert pymunk vec2d to pygame integers"""
    return round(p.x), round(p.y)

def create_edge(space, vs, pos):
    body = pymunk.Body(100, 100,pymunk.Body.DYNAMIC)
    body.position = pos
    
    poly1 = pymunk.Poly(body, vertices=vs)
    space.add(body, poly1)
    return poly1, body

def create_link(space):
    unit = 40
    origin = (200,400)
    pivot1 = add_tuple(origin, (1*unit,0.5*unit))
    pivot2 = add_tuple(pivot1,(2*unit,2.5*unit))
    pivot3 = add_tuple(pivot2, (2*unit, 7.5*unit))
    

    vs0 = [origin, pivot1, origin, pivot1]
    vs1 = [pivot1, add_tuple(pivot1, (1*unit,0.5*unit)), add_tuple(pivot1, (1*unit,0.5*unit)), pivot1]
    vs2 = [add_tuple(pivot1,(1*unit,0.5*unit)), pivot2, pivot2, add_tuple(pivot1,(1*unit,0.5*unit))]
    vs3 = [pivot2, add_tuple(pivot2, (unit, 2*unit)), pivot2, add_tuple(pivot2, (unit, 2*unit))]
    vs4 = [add_tuple(pivot2, (unit, 2*unit)), pivot3,  pivot3, add_tuple(pivot2, (unit, 2*unit))]
    

    poly0, body0 = create_edge(space, vs0, pivot1)
    poly1, body1 = create_edge(space, vs1, pivot1)
    poly2, body2 = create_edge(space, vs2, pivot2)
    pymunk.PinJoint(body1, body2, add_tuple(pivot1,(1*unit,0.5*unit)))
    poly3, body3 = create_edge(space, vs3, pivot2)
    poly4, body4 = create_edge(space, vs4, pivot3)
    pymunk.PinJoint(body3, body4, add_tuple(pivot2, (unit, 2*unit)))

    pymunk.PivotJoint(body0, body1, pivot1)
    pymunk.PivotJoint(body2, body3, pivot2)

    # b0 = space.static_body
    body_st = pymunk.Body(body_type=pymunk.Body.STATIC)
    body_st.position = pivot3
    shape_st = pymunk.Circle(body_st, 50)
    space.add(body_st, shape_st)

    pymunk.PivotJoint(body_st, body4, pivot3)

    
    return poly0, body0, poly1, body1, poly2, body2, poly3, body3, poly4, body4
    
def draw_lines(screen, poly0, body0, poly1, body1, poly2, body2, poly3, body3, poly4, body4):
    
    pv0 = poly0.get_vertices()
    pv0[0] += body0.position
    pv0[1] += body0.position
    pv1 = poly1.get_vertices()
    pv2 = poly2.get_vertices()
    pv1[0] += body1.position
    pv1[1] += body1.position
    pv2[0] += body2.position
    pv2[1] += body2.position
    
    pv3 = poly3.get_vertices()
    pv4 = poly4.get_vertices()
    pv3[0] += body3.position
    pv3[1] += body3.position
    pv4[0] += body4.position
    pv4[1] += body4.position
    
    pygame.draw.lines(screen, (12, 68, 237), False, [pv0[0], pv0[1], pv1[0], pv1[1]], width=5)
    pygame.draw.lines(screen, (28, 214, 11), False, [pv1[0], pv1[1], pv2[0], pv2[1]], width=5)
    pygame.draw.lines(screen, (245, 83, 118), False, [pv3[0], pv3[1], pv4[0], pv4[1]], width=5)
    pygame.draw.circle(screen, (0,0,0), pv4[1], 50)

    


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    clock = pygame.time.Clock()
    space = pymunk.Space()
    space.gravity = (0, 900)
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    pl0, bd0, pl1, bd1, pl2, bd2, pl3, bd3, pl4, bd4 = create_link(space)
    

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        screen.fill((250, 250, 250))
        draw_lines(screen, pl0, bd0, pl1, bd1, pl2, bd2, pl3, bd3, pl4, bd4)
        space.debug_draw(draw_options) 
        space.step(1/50)
        pygame.display.update()
        clock.tick(50)     
        
         

if __name__ == '__main__':
    main()        