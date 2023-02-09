import copy

import numpy as np
from typing import List

class Point:
    def __init__(self, x: float, y: float):
        self._x = x
        self._y = y

    @staticmethod
    def nil_point():
        return Point(-10000, -10000)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, x: float):
        self._x = x

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, y: float):
        self._y = y

    def equal(self, p) -> bool:
        return self._x == p._x and self._y == p._y

    def __equal__(self, p) -> bool:
        return self._x == p._x and self._y == p._y

    def turn_left(self):
        pass

    def is_nil_point(self):
        return self.x == Point.nil_point().x and self.y == Point.nil_point().y

    @staticmethod
    def calc_euclidean_dist(p, q):
        dist = ((p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y)) ** 0.5
        return dist


def point_in_road_network(p: Point) -> bool:
    return (X_MIN <= p.x <= X_MAX and Y_MIN <= p.y <= Y_MAX)

def fall_in_which_intersection(point: Point) -> Point:
    intersection: Point = Point.nil_point()
    for i in range(len(X_OF_INTERSECTIONS)):
        x = X_OF_INTERSECTIONS[i]
        x_min = x - 0.5 * INTERSECTION_DIST
        x_max = x + 0.5 * INTERSECTION_DIST
        if x_min <= point.x <= x_max:
            for j in range(len(Y_OF_INTERSECTIONS)):
                y = Y_OF_INTERSECTIONS[j]
                y_min = y - 0.5 * INTERSECTION_DIST
                y_max = y + 0.5 * INTERSECTION_DIST
                if y_min <= point.y <= y_max:
                    intersection.x = x
                    intersection.y = y
    return intersection

SELECT_TRAIN_TEST = 0  # 0: train, 1: test

PARALLEL = 0  # 0: not parallel, 1: parallel
NUM_CORES = 16

# vehicle's action in intersection
VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT = "straight"
VEHICLE_ACTION_IN_INTERSECTION_LEFT = "left"
VEHICLE_ACTION_IN_INTERSECTION_RIGHT = "right"

VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY = 1.0
VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY = 0.0
VEHICLE_ACTION_IN_INTERSECTION_RIGHT_PROBABILITY = 0.0

# vehicle's movement direction
VEHICLE_MOVEMENT_DIRECTION_TO_EAST = "east"
VEHICLE_MOVEMENT_DIRECTION_TO_WEST = "west"
VEHICLE_MOVEMENT_DIRECTION_TO_NORTH = "north"
VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH = "south"

VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT = {"east": 0, "west": 1, "north": 2, "south": 3, "None": 4}
INDEX_VEHICLE_MOVEMENT_DIRECTION_DICT = {"0": "east", "1": "west", "2": "north", "3": "south", "4": None}
VEHICLE_SPLIT_DICT = {"east": 0, "west": 1, "north": 2, "south": 3, "in": 4}

TRAFFIC_LIGHT_EASTWEST_PASS = 0
TRAFFIC_LIGHT_EASTWESTYELLOW_NORTHSOUTHRED = 1
TRAFFIC_LIGHT_NORTHSOUTH_PASS = 2
TRAFFIC_LIGHT_EASTWESTRED_NORTHSOUTHYELLOW = 3

BLOCK_DIST = 4  # dist between two adjacent blocks
assert type(BLOCK_DIST) == int and BLOCK_DIST % 2 == 0



# 0: tony model (an intersection, 5 blocks),       dqn
# 1: realistic model (an intersection, 33 blocks), ddpg
# 2: grid network (5 * 10 road network),           ddpg
ROAD_MODEL = 2
assert ROAD_MODEL in [0, 1, 2]

if ROAD_MODEL in [0, 1]:
    BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS = 0
else:  # ROAD_MODEL == 2
    BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS = 10

if ROAD_MODEL == 0:
    INTERSECTION_DIST = BLOCK_DIST
else:  # ROAD_MODEL == 1, 2
    INTERSECTION_DIST = 2 * BLOCK_DIST

# INTERSECTIONS_NUMS = [5, 10]  # 5 X 10 grid network

# X_MIN X_MAX Y_MIN Y_MAX denotes the center of block or intersection

INTERSECTIONS = []
if ROAD_MODEL == 0:
    X_OF_INTERSECTIONS = [0]
    Y_OF_INTERSECTIONS = [0]

    p0 = Point(0, 0)
    INTERSECTIONS: List[Point] = [p0]
    INTERSECTIONS_MATRIX = [p0]

    X_MIN = -1 * BLOCK_DIST
    X_MAX = 1 * BLOCK_DIST
    Y_MIN = -1 * BLOCK_DIST
    Y_MAX = 1 * BLOCK_DIST

    # Here, blocks do not include intersections
    p1 = Point(0, BLOCK_DIST)
    p2 = Point(-BLOCK_DIST, 0)
    p3 = Point(0, -BLOCK_DIST)
    p4 = Point(BLOCK_DIST, 0)
    BLOCKS = [p0, p1, p2, p3, p4]
    BLOCKS_FOR_UAV = copy.deepcopy(BLOCKS)
elif ROAD_MODEL == 1:
    X_OF_INTERSECTIONS = [0]
    Y_OF_INTERSECTIONS = [0]

    p1 = Point(0, 0)
    INTERSECTIONS: List[Point] = [p1]
    INTERSECTIONS_MATRIX = [p1]

    X_MIN = -4.5 * BLOCK_DIST
    X_MAX = 4.5 * BLOCK_DIST
    Y_MIN = -4.5 * BLOCK_DIST
    Y_MAX = 4.5 * BLOCK_DIST

    # Here, blocks do not include intersections
    x_east = list(range(int(1.5 * BLOCK_DIST), int(4.5 * BLOCK_DIST + 1), BLOCK_DIST))
    x_west = list(range(int(-1.5 * BLOCK_DIST), int(-4.5 * BLOCK_DIST - 1), -BLOCK_DIST))
    y_north = list(range(int(1.5 * BLOCK_DIST), int(4.5 * BLOCK_DIST + 1), BLOCK_DIST))
    y_south = list(range(int(-1.5 * BLOCK_DIST), int(-4.5 * BLOCK_DIST - 1), -BLOCK_DIST))
    p0 = Point(0, 0)
    p1 = Point(-0.5 * BLOCK_DIST, -1.5 * BLOCK_DIST)
    p2 = Point(0.5 * BLOCK_DIST, -1.5 * BLOCK_DIST)
    p3 = Point(1.5 * BLOCK_DIST, -0.5 * BLOCK_DIST)
    p4 = Point(1.5 * BLOCK_DIST, 0.5 * BLOCK_DIST)
    p5 = Point(0.5 * BLOCK_DIST, 1.5 * BLOCK_DIST)
    p6 = Point(-0.5 * BLOCK_DIST, 1.5 * BLOCK_DIST)
    p7 = Point(-1.5 * BLOCK_DIST, 0.5 * BLOCK_DIST)
    p8 = Point(-1.5 * BLOCK_DIST, -0.5 * BLOCK_DIST)
    BLOCKS = []
    BLOCKS_FOR_UAV = [p0, p1, p2, p3, p4, p5, p6, p7, p8]
    for x in x_east:
        p1 = Point(x, -0.5 * BLOCK_DIST)
        p2 = Point(x, 0.5 * BLOCK_DIST)
        BLOCKS.append(p1)
        BLOCKS.append(p2)
    for x in x_west:
        p1 = Point(x, -0.5 * BLOCK_DIST)
        p2 = Point(x, 0.5 * BLOCK_DIST)
        BLOCKS.append(p1)
        BLOCKS.append(p2)
    for y in y_north:
        p1 = Point(-0.5 * BLOCK_DIST, y)
        p2 = Point(0.5 * BLOCK_DIST, y)
        BLOCKS.append(p1)
        BLOCKS.append(p2)
    for y in y_south:
        p1 = Point(-0.5 * BLOCK_DIST, y)
        p2 = Point(0.5 * BLOCK_DIST, y)
        BLOCKS.append(p1)
        BLOCKS.append(p2)
        pass
    # duplicate = [0] * len(BLOCKS_FOR_UAV)
    # for i in range(len(BLOCKS_FOR_UAV) - 1):
    #     if duplicate[i] == 1:
    #         continue
    #     for j in range(i + 1, len(BLOCKS_FOR_UAV)):
    #         if BLOCKS_FOR_UAV[i].equal(BLOCKS_FOR_UAV[j]):
    #             duplicate[j] = 1
    # tmp_blocks_for_uav = []
    # for i in range(len(BLOCKS_FOR_UAV)):
    #     if duplicate[i] == 0:
    #         tmp_blocks_for_uav.append(BLOCKS_FOR_UAV[i])
    # BLOCKS_FOR_UAV = tmp_blocks_for_uav
elif ROAD_MODEL == 2:
    X_OF_INTERSECTIONS = list((BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) * np.array(range(-4, 6)))
    Y_OF_INTERSECTIONS = list((BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) * np.array(range(-2, 3)))

    INTERSECTIONS: List[Point] = []
    for x in X_OF_INTERSECTIONS:
        for y in Y_OF_INTERSECTIONS:
            p = Point(x, y)
            INTERSECTIONS.append(p)

    INTERSECTIONS_MATRIX = []
    for y in Y_OF_INTERSECTIONS:
        row = []
        for x in X_OF_INTERSECTIONS:
            row.append((y, x))
        INTERSECTIONS_MATRIX.append(row)

    X_MIN = int(-5 * (BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) + 1.5 * BLOCK_DIST)
    X_MAX = int(6 * (BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) - 1.5 * BLOCK_DIST)
    Y_MIN = int(-3 * (BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) + 1.5 * BLOCK_DIST)
    Y_MAX = int(3 * (BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS * BLOCK_DIST + INTERSECTION_DIST) - 1.5 * BLOCK_DIST)

    # check if the block is realted to intersections, i.e., the x, or y of block falls the region of intersections.
    def find_related_intersection(p: Point):
        find = False
        for x in X_OF_INTERSECTIONS:
            x_min = x - 0.5 * INTERSECTION_DIST
            x_max = x + 0.5 * INTERSECTION_DIST
            if x_min <= p.x <= x_max:
                find = True
                break
        for y in Y_OF_INTERSECTIONS:
            y_min = y - 0.5 * INTERSECTION_DIST
            y_max = y + 0.5 * INTERSECTION_DIST
            if y_min <= p.y <= y_max:
                find = True
                break
        return find

    BLOCKS = []
    BLOCKS_FOR_UAV = None  # not designed for uav
    for x in range(X_MIN, X_MAX + 1, BLOCK_DIST):
        for y in range(Y_MIN, Y_MAX + 1, BLOCK_DIST):
            p = Point(x, y)
            q = fall_in_which_intersection(p)
            if q.is_nil_point() and find_related_intersection(p):
                BLOCKS.append(p)
    pass
pass

# uav parameters
DIST_UAV_MOVE_DOWNWARD_UPWARD = 5  # meter, dist if the uav moves downward or upward
UAV_MIN_HEIGHT = 100  # meter
UAV_MAX_HEIGHT = 500  # meter
if ROAD_MODEL == 0:
    UAV_HORIZONTAL_MOVE_ACTION_NUM = 8
elif ROAD_MODEL == 1:
    UAV_HORIZONTAL_MOVE_ACTION_NUM = 12
else:
    UAV_HORIZONTAL_MOVE_ACTION_NUM = None
UAV_VERTICAL_MOVE_ACTION_NUM = 3
UAV_VERTICAL_MOVE_ACTION_DOWNWARD = 0
UAV_VERTICAL_MOVE_ACTION_STAY = 1
UAV_VERTICAL_MOVE_ACTION_UPWARD = 2
# communication parameters
ALPHA1 = 0.28
ALPHA2 = 3
BETA1 = 0.01
BETA2 = 2 ** (-130 * 0.1) * 10 ** -3
LOS = 1  # channel state
NLOS = 0  # channel state
TOTAL_POWER = 6  # unit: W
TOTAL_CHANNELS_NUM = 10
BANDWIDTH_OF_ONE_CHANNEL = 100 * 1000  # 100 KHz
SIGMA_SQUARE = 3


# DRL params
START_TIMESLOT = 0
END_TIMESLOT = 10000

START_EPISODE = 0
END_EPISODE = 1000

START_STEP = 0
END_STEP = 200

BINOMIAL_PARAM_EAST_WEST = 0.7  # binomial_distribution_param PROBABILITY
BINOMIAL_PARAM_NORTH_SOUTH = 0.2  # binomial_distribution_param PROBABILITY

NUM_SECONDS_ONE_TIMESLOT = 6  # one timeslot occupies 6 seconds






