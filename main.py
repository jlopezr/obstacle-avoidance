import pygame
import math
import numpy as np

def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

class Robot:
    def __init__(self, startpos, width):
        self.m2p = 3779.52 # from meters to pixels
        self.w = width # robot width

        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p

        self.min_obs_dist = 100
        self.count_down = 5 # seconds

    def avoid_obstacles(self, point_cloud, dt):
        closest_obs = None
        dist = np.inf

        if len(point_cloud) > 0:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)

            if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                self.count_down -= dt
                self.move_backward()
            else:
                # reset countdown
                self.count_down = 5
                # move forward
                self.move_forward()

    def move_backward(self):
        self.vl = -self.minspeed/2
        self.vr = -self.minspeed

    def move_forward(self):
        self.vl = self.minspeed
        self.vr = self.minspeed

    def kinematics(self, dt):
        self.x += (self.vl + self.vr)/2 * math.cos(self.heading) * dt
        self.y -= (self.vl + self.vr)/2 * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl)/self.w * dt

        if self.heading > 2*math.pi or self.heading < -2*math.pi:
            self.heading = 0

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)    
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()

        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)

        # load images
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)

        # window
        pygame.display.set_caption("Obstacle Avoidance")
        self.height, self.width = dimensions
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map.blit(self.map_img, (0, 0))
        
    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)

class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map = map
        self.map_width, self.map_height = self.map.get_size()

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 10, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0,100):
                u = i/100
                x = int(x2 * u + x1 * (1-u))
                y = int(y2 * u + y1 * (1-u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 205))
                    if color == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles



MAP_DIMENSIONS = (600,1200)
gfx = Graphics(MAP_DIMENSIONS, "robot.png", "map.png")

start = (200, 200)
robot = Robot(start, 0.01*3779.52)

sensor_range = 250, math.radians(40)
ultra_sonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()

    gfx.map.blit(gfx.map_img, (0, 0))
    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading)

    robot.avoid_obstacles(point_cloud, dt)
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()

