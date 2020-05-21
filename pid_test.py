import pymunk
import pygame
import random
import sys
from pymunk.pygame_util import DrawOptions, to_pygame

from simple_pid import PID

width = 600
height = 600

REFRESH = 50.0

WHITE = (255, 255, 255)


class Drone:
    def __init__(self, position, space):
        self.mass = 1
        self.shape = pymunk.Poly.create_box(None, size=(50, 10))
        self.moment = pymunk.moment_for_poly(self.mass, self.shape.get_vertices())
        self.body = pymunk.Body(self.mass, self.moment)
        self.shape.body = self.body
        leg1 = pymunk.Segment(self.body, (-20, -30), (-10, 0), 3)  # 2
        leg2 = pymunk.Segment(self.body, (20, -30), (10, 0), 3)

        self.shape.body.position = position
        space.add(self.shape, self.body, leg1, leg2)


class Ground:
    def __init__(self, space):
        self.body = pymunk.Body(0, 0, body_type=pymunk.Body.STATIC)
        self.shape = pymunk.Poly.create_box(self.body, (width, 10))
        self.shape.body.position = (width // 2, 10)
        space.add(self.shape, self.body)


def main():
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()

    tunings = (27, 2, 10)
    pid = PID(*tunings, output_limits=(-8000, 8000), proportional_on_measurement=True)
    pid.sample_time = 1 / REFRESH

    draw_options = DrawOptions(screen)
    pymunk.pygame_util.positive_y_is_up = True

    font = pygame.font.SysFont('Lucida Console', 11)

    space = pymunk.Space()
    space.gravity = 0, -100
    x = random.randint(120, 380)
    drone = Drone((x, 450), space)
    ground = Ground(space)

    setpoint = 450
    ticks = 0
    while True:
        for event in pygame.event.get():
            # print(event)
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    setpoint += 5
                elif event.key == pygame.K_DOWN:
                    setpoint -= 5
                elif event.key == pygame.K_ESCAPE:
                    sys.exit(0)
                elif event.key == pygame.K_1:
                    p, i, d = pid.tunings
                    pid.tunings = (p + 0.5, i, d)
                elif event.key == pygame.K_q:
                    p, i, d = pid.tunings
                    pid.tunings = (p - 0.5, i, d)
                elif event.key == pygame.K_2:
                    p, i, d = pid.tunings
                    pid.tunings = (p, i + 0.5, d)
                elif event.key == pygame.K_w:
                    p, i, d = pid.tunings
                    pid.tunings = (p, i - 0.5, d)
                elif event.key == pygame.K_3:
                    p, i, d = pid.tunings
                    pid.tunings = (p, i, d + 0.5)
                elif event.key == pygame.K_e:
                    p, i, d = pid.tunings
                    pid.tunings = (p, i, d - 0.5)
            elif event.type == pygame.MOUSEBUTTONUP:
                setpoint = 600 - event.pos[1]
                # ticks=0

        screen.fill((0, 0, 0))

        error = drone.body.position.y - setpoint
        output = pid(error)

        space.debug_draw(draw_options)

        # if drone.shape.body.position.y < setpoint:
        if True:  # output > 0:
            # drone.shape.body.apply_force_at_local_point((0, 340), (0, 0))
            drone.shape.body.apply_force_at_local_point((0, output), (0, 0))
            c = to_pygame(drone.body.position, screen)
            pygame.draw.aaline(screen, (0, 128, 255, .75), c, (c[0], c[1] - output // 10))

        pygame.draw.aaline(screen, (128, 255, 128, .125),
                           to_pygame((50, setpoint), screen),
                           to_pygame((600 - 50, setpoint), screen))

        # pygame.draw.aaline(screen, (255, 255, 255, .5), to_pygame((0,0), screen), to_pygame((100,100), screen))

        textsurface = font.render(f"pos:{drone.body.position.y:7.1f} set:{setpoint} error:{error:7.1f} o:{output:7.1f}", False, WHITE)
        screen.blit(textsurface, (10, 10))
        textsurface = font.render(f"pid:{['{:7.1f}'.format(p) for p in pid.components]}", False, WHITE)
        screen.blit(textsurface, (10, 25))
        textsurface = font.render(f"tun:{['{:7.1f}'.format(p) for p in pid.tunings]}", False, WHITE)
        screen.blit(textsurface, (10, 40))
        textsurface = font.render(f"ticks:{ticks}", False, WHITE)
        screen.blit(textsurface, (10, 55))

        space.step(1 / REFRESH)
        pygame.display.update()
        clock.tick(REFRESH)

        pygame.display.set_caption(f"{drone.body.position.y:.1f} {setpoint}")

        ticks += 1
        # if ticks >= 300: #random.randint(500, 1000):
        #     # setpoint = random.randint(100, 500)
        #     if setpoint == 500:
        #         setpoint = 100
        #     else:
        #         setpoint = 500
        #     ticks = 0


if __name__ == '__main__':
    sys.exit(main())
