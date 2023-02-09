# fun.py
import numpy as np
import copy
import random
from config import X_OF_INTERSECTIONS
from config import Y_OF_INTERSECTIONS
from config import INTERSECTIONS

from typing import List
from typing import Union
# from Vehicle import Vehicle

# vehicles: sorted by timeslot
import config

from config import BINOMIAL_PARAM_EAST_WEST
from config import BINOMIAL_PARAM_NORTH_SOUTH

from config import START_TIMESLOT
from config import END_TIMESLOT
from config import VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
from config import VEHICLE_ACTION_IN_INTERSECTION_LEFT
from config import VEHICLE_ACTION_IN_INTERSECTION_RIGHT
from config import X_MIN
from config import X_MAX
from config import Y_MIN
from config import Y_MAX
from config import X_OF_INTERSECTIONS
from config import Y_OF_INTERSECTIONS
from config import BLOCK_DIST
from config import ROAD_MODEL
from config import TRAFFIC_LIGHT_EASTWEST_PASS
from config import TRAFFIC_LIGHT_EASTWESTYELLOW_NORTHSOUTHRED
from config import TRAFFIC_LIGHT_NORTHSOUTH_PASS
from config import TRAFFIC_LIGHT_EASTWESTRED_NORTHSOUTHYELLOW
from config import VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
from config import VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
from config import VEHICLE_MOVEMENT_DIRECTION_TO_EAST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_WEST

from vehicle import Vehicle
from config import Point


# Is the point p on the segment p1-p2?
def is_on_segment(p: Point, p1: Point, p2: Point):
    on = False
    x_min = min(p1.x, p2.x)
    x_max = max(p1.x, p2.x)
    y_min = min(p1.y, p2.y)
    y_max = max(p1.y, p2.y)
    if x_min <= p.x <= x_max and y_min <= p.y <= y_max:
        if (p.y - p1.y) * (p1.x - p2.x) == (p1.y - p2.y) * (p.x - p1.x):
            on = True
    return on



# def is_point_in_road_network(point: Point) -> bool:
#     return (X_MIN <= point.x <= X_MAX and Y_MIN <= point.y <= Y_MAX)








# def calc_point(block: int) -> Point:
#     x = 0
#     y = 0
#     if ROAD_MODEL == 0:
#         if block in [2, 4]:
#             y = 0
#             if block == 2:
#                 x = -BLOCK_DIST
#             elif block == 4:
#                 x = BLOCK_DIST
#         elif block in [1, 3]:
#             x = 0
#             if block == 1:
#                 y = BLOCK_DIST
#             elif block == 3:
#                 y = -BLOCK_DIST
#     elif ROAD_MODEL == 1:
#         if block in [4, 12, 20, 28, 7, 15, 23, 31]:
#             y = 0.5 * BLOCK_DIST
#         if block in [3, 11, 19, 27, 8, 16, 24, 32]:
#             y = -0.5 * BLOCK_DIST
#         if block in [6, 14, 22, 30, 1, 9, 17, 25]:
#             x = -0.5 * BLOCK_DIST
#         if block in [5, 13, 21, 29, 2, 10, 18, 26]:
#             x = 0.5 * BLOCK_DIST
#         if block in [7, 8]:
#             x = -1.5 * BLOCK_DIST
#         if block in [15, 16]:
#             x = -2.5 * BLOCK_DIST
#         if block in [23, 24]:
#             x = -3.5 * BLOCK_DIST
#         if block in [31, 32]:
#             x = -4.5 * BLOCK_DIST
#         if block in [3, 4]:
#             x = 1.5 * BLOCK_DIST
#         if block in [11, 12]:
#             x = 2.5 * BLOCK_DIST
#         if block in [19, 20]:
#             x = 3.5 * BLOCK_DIST
#         if block in [27, 28]:
#             x = 4.5 * BLOCK_DIST
#         if block in [5, 6]:
#             y = 1.5 * BLOCK_DIST
#         if block in [13, 14]:
#             y = 2.5 * BLOCK_DIST
#         if block in [21, 22]:
#             y = 3.5 * BLOCK_DIST
#         if block in [29, 30]:
#             y = 4.5 * BLOCK_DIST
#         if block in [1, 2]:
#             y = -1.5 * BLOCK_DIST
#         if block in [9, 10]:
#             y = -2.5 * BLOCK_DIST
#         if block in [17, 18]:
#             y = -3.5 * BLOCK_DIST
#         if block in [25, 26]:
#             y = -4.5 * BLOCK_DIST
#     p = Point(x, y)
#     return p

def calc_euclidean_dist(delta1: float, delta2: float):
    return (delta1 ** 2 + delta2 ** 2) ** 0.5

# def calc_horizontal_dist_between_blocks(block1: int, block2: int):
#     point1 = calc_point(block1)
#     point2 = calc_point(block2)
#     dist = ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5
#     return dist







# def generate_vehicles(start_step: int, end_step: int):
#     seed_ = 50
#     for step in range(start_step, end_step + 1):
#         if



