import time

import config
import numpy as np
import copy
import random
from typing import List
from typing import Union
from typing import Optional
from multiprocessing import Pool
from config import PARALLEL
from config import NUM_CORES
from config import point_in_road_network
from config import VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
from config import VEHICLE_ACTION_IN_INTERSECTION_LEFT
from config import VEHICLE_ACTION_IN_INTERSECTION_RIGHT
from config import BLOCK_DIST
from config import INTERSECTION_DIST
from config import ROAD_MODEL
from config import VEHICLE_MOVEMENT_DIRECTION_TO_EAST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_WEST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
from config import VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
from config import BLOCK_NUM_BETWEEN_TWO_ADJ_INTERSECTIONS
from config import X_OF_INTERSECTIONS
from config import Y_OF_INTERSECTIONS
from config import INTERSECTIONS_MATRIX
from config import Point
from config import VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT
from config import INDEX_VEHICLE_MOVEMENT_DIRECTION_DICT
from config import VEHICLE_SPLIT_DICT
# from fun import calc_new_direction
# from fun import calc_new_point
from config import INTERSECTIONS
from config import BINOMIAL_PARAM_EAST_WEST
from config import BINOMIAL_PARAM_NORTH_SOUTH
from config import START_STEP
from config import END_STEP
from config import VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
from config import VEHICLE_ACTION_IN_INTERSECTION_LEFT
from config import VEHICLE_ACTION_IN_INTERSECTION_RIGHT
from config import VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY
from config import VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY
from config import VEHICLE_ACTION_IN_INTERSECTION_RIGHT_PROBABILITY

from traffic_light import calc_traffic_light_states_in_intersections
from config import X_MIN
from config import X_MAX
from config import Y_MIN
from config import Y_MAX
from config import TRAFFIC_LIGHT_EASTWEST_PASS
from config import TRAFFIC_LIGHT_EASTWESTYELLOW_NORTHSOUTHRED
from config import TRAFFIC_LIGHT_NORTHSOUTH_PASS
from config import TRAFFIC_LIGHT_EASTWESTRED_NORTHSOUTHYELLOW
from config import fall_in_which_intersection
# calc new direction by curr direction and action in an intersection.
def calc_new_direction(direction: str, action: str):
    if direction == VEHICLE_MOVEMENT_DIRECTION_TO_EAST:
        if action == VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_EAST
        elif action == VEHICLE_ACTION_IN_INTERSECTION_LEFT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
        elif action == VEHICLE_ACTION_IN_INTERSECTION_RIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_WEST:
        if action == VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_WEST
        elif action == VEHICLE_ACTION_IN_INTERSECTION_LEFT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
        elif action == VEHICLE_ACTION_IN_INTERSECTION_RIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_NORTH:
        if action == VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
        elif action == VEHICLE_ACTION_IN_INTERSECTION_LEFT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_WEST
        elif action == VEHICLE_ACTION_IN_INTERSECTION_RIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_EAST
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH:
        if action == VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
        elif action == VEHICLE_ACTION_IN_INTERSECTION_LEFT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_EAST
        elif action == VEHICLE_ACTION_IN_INTERSECTION_RIGHT:
            new_direction = VEHICLE_MOVEMENT_DIRECTION_TO_WEST
    return new_direction

# def fall_in_which_intersection(point: Point) -> Point:
#     intersection: Point = Point.nil_point()
#     for i in range(len(X_OF_INTERSECTIONS)):
#         x = X_OF_INTERSECTIONS[i]
#         x_min = x - 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
#         x_max = x + 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
#         if x_min <= point.x <= x_max:
#             for j in range(len(Y_OF_INTERSECTIONS)):
#                 y = Y_OF_INTERSECTIONS[j]
#                 y_min = y - 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
#                 y_max = y + 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
#                 if y_min <= point.y <= y_max:
#                     intersection.x = x
#                     intersection.y = y
#     return intersection

# in point, and knows the direction, calc the new point
# if moving one step (may not be equal to one BLOCK_DIST if the next is intersection),
# which may move to an intersection
def calc_new_point(point: Point, direction: str) -> List[Union[Point, bool]]:
    from config import fall_in_which_intersection
    change_to_intersection = False
    new_point = copy.deepcopy(point)

    if is_point_intersection(point) and ROAD_MODEL in [1, 2]:
        delta_dist = 0.5 * INTERSECTION_DIST + 0.5 * BLOCK_DIST
    else:
        delta_dist = 1 * BLOCK_DIST
    if direction == VEHICLE_MOVEMENT_DIRECTION_TO_EAST:
        new_point.x += delta_dist
        if is_point_intersection(point) and ROAD_MODEL in [1, 2]:
            new_point.y -= 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_WEST:
        new_point.x -= delta_dist
        if is_point_intersection(point) and ROAD_MODEL in [1, 2]:
            new_point.y += 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_NORTH:
        new_point.y += delta_dist
        if is_point_intersection(point) and ROAD_MODEL in [1, 2]:
            new_point.x += 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH:
        new_point.y -= delta_dist
        if is_point_intersection(point) and ROAD_MODEL in [1, 2]:
            new_point.x -= 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
    if new_point.x < X_MIN or new_point.x > X_MAX or new_point.y < Y_MIN or new_point.y > Y_MAX:
        new_point = Point.nil_point()
    else:
        intersection = fall_in_which_intersection(new_point)
        if not intersection.is_nil_point():
            new_point = intersection
            change_to_intersection = True
    return new_point, change_to_intersection

# action_in_intersection is an int, when the vehicle crosses an intersection, the index_of_intersection +1
class Vehicle:
    def __init__(self,
                 id_: int,
                 appearing_step: int,
                 startpoint: Point,
                 action_in_intersection: str,):
        self._id_: int = id_
        self._appearing_step: int = appearing_step
        self._startpoint: Point = startpoint
        self._action_in_intersection: str = action_in_intersection
        self._points: List[Point] = []
        self._index_of_point: int = 0
        self._directions: List[str] = []
        self._index_of_direction: int = 0
        self._will_move: bool = False  # will move in the future
        self._should_move: bool = False  # should_move in the current step

        #  the intersection where the vehicle may change the direction.
        #  assume there is only one pivot_intersection for one vehicle
        self._pivot_intersection: Point = Point.nil_point()

    @property
    def id_(self):
        return self._id_

    @id_.setter
    def id_(self, id_: int):
        self._id_ = id_

    @property
    def appearing_step(self):
        return self._appearing_step

    @appearing_step.setter
    def appearing_step(self, appearing_step: int):
        self._appearing_step = appearing_step

    @property
    def directions(self):
        return self._directions

    @directions.setter
    def directions(self, directions: List[str]):
        self._directions = directions

    @property
    def direction(self):
        return self._directions[self.index_of_direction]

    @property
    def point(self):
        return self._points[self._index_of_point]

    @property
    def prepoint(self):
        if self._index_of_point == 0:
            return Point.nil_point()
        return self._points[self._index_of_point]

    @property
    def nextpoint(self):
        if self._index_of_point == len(self.points) - 1:
            return Point.nil_point()
        return self._points[self._index_of_point + 1]

    @property
    def next2point(self):
        if self._index_of_point >= len(self.points) - 2:
            return Point.nil_point()
        return self._points[self._index_of_point + 2]

    @property
    def startpoint(self):
        return self._startpoint

    @startpoint.setter
    def startpoint(self, startpoint: Point):
        self._startpoint = startpoint

    @property
    def pivot_intersection(self):
        return self._pivot_intersection

    @pivot_intersection.setter
    def pivot_intersection(self, pivot_intersection: Point):
        self._pivot_intersection = pivot_intersection

    # @point.setter
    # def point(self, point: Point):
    #     self._point = point

    # @property
    # def nextpoint_if_move(self):
    #     return self._points[self._index_of_point + 1] if self._index_of_point < len(self._points) else Point.nil_point()

    # @nextpoint_if_move.setter
    # def nextpoint_if_move(self, nextpoint_if_move: Point):
    #     self._nextpoint_if_move = nextpoint_if_move


    @property
    def points(self):
        return self._points

    @points.setter
    def points(self, points: List[Point]):
        self._points = points


    @property
    def index_of_point(self):
        return self._index_of_point

    @index_of_point.setter
    def index_of_point(self, index_of_point: int):
        self._index_of_point = index_of_point

    @property
    def index_of_direction(self):
        return self._index_of_direction

    @index_of_direction.setter
    def index_of_direction(self, index_of_direction: int):
        self._index_of_direction = index_of_direction

    @property
    def action_in_intersection(self):
        return self._action_in_intersection

    @action_in_intersection.setter
    def action_in_intersection(self, action_in_intersection: str):
        self._action_in_intersection = action_in_intersection

    @property
    def will_move(self):
        return self._will_move

    @will_move.setter
    def will_move(self, will_move: bool):
        self._will_move = will_move

    @property
    def should_move(self):
        return self._should_move

    @should_move.setter
    def should_move(self, should_move: bool):
        self._should_move = should_move


    def in_road_network(self) -> bool:
        return (X_MIN <= self.point.x <= X_MAX and Y_MIN <= self.point.y <= Y_MAX)

    def calc_toward_intersection_index(self):
        intersection_index = None
        for i in range(self.index_of_point, len(self.points)):
            if intersection_index is not None:
                break
            for j in range(len(INTERSECTIONS)):
                if(self.points[i].equal(INTERSECTIONS[j])):
                    intersection_index = j
                    break
        return intersection_index


    # calc direction by self.points[0]
    def calc_init_direction(self) -> str:
        direction = None
        if self.points[0].x == X_MIN:
            direction = VEHICLE_MOVEMENT_DIRECTION_TO_EAST
        elif self.points[0].x == X_MAX:
            direction = VEHICLE_MOVEMENT_DIRECTION_TO_WEST
        elif self.points[0].y == Y_MIN:
            direction = VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
        elif self.points[0].y == Y_MAX:
            direction = VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
        return direction

    # self.startpoint should be known
    # update self.directions, self.pivot_intersection, and self.points
    def update_points(self):
        assert len(self.directions) == 0
        i_max = len(X_OF_INTERSECTIONS) - 1
        j_max = len(Y_OF_INTERSECTIONS) - 1
        if self.startpoint.x == X_MIN:
            self.directions.append(VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
            i = np.random.randint(0, i_max + 1)
            self.pivot_intersection.y = self.startpoint.y + 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
            self.pivot_intersection.x = X_OF_INTERSECTIONS[i]
        elif self.startpoint.x == X_MAX:
            self.directions.append(VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
            i = np.random.randint(0, i_max + 1)
            self.pivot_intersection.y = self.startpoint.y - 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
            self.pivot_intersection.x = X_OF_INTERSECTIONS[i]
        elif self.startpoint.y == Y_MIN:
            self.directions.append(VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
            j = np.random.randint(0, j_max + 1)
            self.pivot_intersection.x = self.startpoint.x - 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
            self.pivot_intersection.y = Y_OF_INTERSECTIONS[j]
        elif self.startpoint.y == Y_MAX:
            self.directions.append(VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
            j = np.random.randint(0, j_max + 1)
            self.pivot_intersection.x = self.startpoint.x + 0.5 * (INTERSECTION_DIST - BLOCK_DIST)
            self.pivot_intersection.y = Y_OF_INTERSECTIONS[j]

        new_direction = calc_new_direction(direction=self.directions[0], action=self.action_in_intersection)
        if new_direction != self.directions[0]:
            self.directions.append(new_direction)

        point = self.startpoint
        index_of_direction = 0
        direction = self.directions[index_of_direction]
        self.points.append(point)

        while True:
            new_point, change_to_intersection = calc_new_point(point=point, direction=direction)
            if change_to_intersection and new_point.equal(self.pivot_intersection):
                if len(self.directions) >= 2:
                    index_of_direction = 1
                    direction = self.directions[index_of_direction]
            self.points.append(new_point)
            point = copy.deepcopy(new_point)
            if point.is_nil_point():
                break
        return

    # executed after update_should_move()
    def move(self):
        if self.point.is_nil_point():
            self.will_move = False
            self.should_move = False
            return
        if not self.will_move or not self.should_move:
            return
        if self.point.equal(self.pivot_intersection):
            if self.index_of_direction + 1 < len(self.directions):
                self.index_of_direction += 1
        self.index_of_point += 1
        return

# filter vehicles. select vehicles whose appearing_step <= step.
# For the vehicles that has the same point, only select one and remove others
def filter_vehicles(vs: List[Vehicle], step: int) -> List[Vehicle]:
    vehicles = []
    indices_remove = []
    for i in range(len(vs)):
        if i in indices_remove:
            continue
        for j in range(i + 1, len(vs)):
            if j in indices_remove:
                continue
            intersection = fall_in_which_intersection(vs[i].point)
            if vs[i].point.equal(vs[j].point) and intersection.is_nil_point():
                indices_remove.append(j)

    for i in range(len(vs)):
        if i not in indices_remove and vs[i].will_move and vs[i].appearing_step <= step:
            v = copy.deepcopy(vs[i])
            vehicles.append(v)
    return vehicles

def calc_should_pass_intersection(movement_direction: str, traffic_light_state: int) -> bool:
    res = False
    if movement_direction in [VEHICLE_MOVEMENT_DIRECTION_TO_EAST, VEHICLE_MOVEMENT_DIRECTION_TO_WEST] \
        and traffic_light_state == TRAFFIC_LIGHT_EASTWEST_PASS:
        res = True
    if movement_direction in [VEHICLE_MOVEMENT_DIRECTION_TO_NORTH, VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH] \
        and traffic_light_state == TRAFFIC_LIGHT_NORTHSOUTH_PASS:
        res = True
    return res

def calc_should_move(vehicle: Vehicle, front_vehicle: Vehicle, intersection_index: int, traffic_light_states: List[int]) -> bool:
    res = False
    if intersection_index is None:
        res = True
    elif is_point_intersection(vehicle.point):
        res = True
    elif front_vehicle is None:
        if is_point_intersection(vehicle.nextpoint):  # the front is intersection
            if calc_should_pass_intersection(vehicle.direction, traffic_light_states[intersection_index]):
                res = True
        else:
            res = True
    else:
        dist = abs(vehicle.point.x - front_vehicle.point.x) + abs(vehicle.point.y - front_vehicle.point.y)
        if dist >= 2 * BLOCK_DIST:
            res = True
        elif front_vehicle.should_move:
            res = True
    return res

# cur_flow is known, traffic_light_states is given,
# let cur_flow move, then add vehicles in their appearing positions,
# new flow of the next step is obtained by cur_flow (after move) and appearing vehicles (not move)
def calc_new_flow_by_adding_vehicles_in_a_step(vehicles: List[Vehicle],
                                               cur_flow: List[List[Vehicle]],
                                               traffic_light_states: List[int]
                                               ) -> List[List[Vehicle]]:
    new_flow: List[List[Vehicle]] = copy.deepcopy(cur_flow)
    # vehicles in a nilintersection move
    for i in range(len(new_flow[-1])):
        new_flow[-1][i].should_move = True
        new_flow[-1][i].move()
    new_flow[-1] = [elem for i, elem in enumerate(new_flow[-1]) if not elem.point.equal(Point.nil_point())]
    for i in range(len(new_flow) - 2, -1, -1):
        # calc intersection_index
        intersection_index = i // 5
        # calc direction
        direction = INDEX_VEHICLE_MOVEMENT_DIRECTION_DICT[str(i % 5)]
        vehicle_ids_remove_put_to_this_intersection = []
        vehicles_put_to_this_intersection = []
        vehicle_ids_remove_put_to_other_intersectionflow = []
        vehicles_put_to_other_intersectionflow = []
        vehicle_ids_remove_put_to_nilintersection = []
        for j in range(len(new_flow[i])):
            front_vehicle = new_flow[i][j - 1] if j >= 1 else None
            new_flow[i][j].should_move = calc_should_move(new_flow[i][j], front_vehicle, intersection_index, traffic_light_states)
            cur_point = copy.deepcopy(new_flow[i][j].point)
            next_point = copy.deepcopy(new_flow[i][j].nextpoint)
            if new_flow[i][j].should_move:
                new_flow[i][j].move()
                toward_intersection_index = new_flow[i][j].calc_toward_intersection_index()
                if toward_intersection_index is None:
                    vehicle_ids_remove_put_to_nilintersection.append(new_flow[i][j].id_)
                    v = copy.deepcopy(new_flow[i][j])
                    new_flow[-1].append(v)
                elif not is_point_intersection(cur_point) and is_point_intersection(next_point):
                    vehicle_ids_remove_put_to_this_intersection.append(new_flow[i][j].id_)
                    v = copy.deepcopy(new_flow[i][j])
                    vehicles_put_to_this_intersection.append(v)
                elif is_point_intersection(cur_point):
                    vehicle_ids_remove_put_to_other_intersectionflow.append(new_flow[i][j].id_)
                    v = copy.deepcopy(new_flow[i][j])
                    vehicles_put_to_other_intersectionflow.append(v)
        new_flow[i] = [elem for _, elem in enumerate(new_flow[i])
                       if elem.id_ not in vehicle_ids_remove_put_to_nilintersection
                       + vehicle_ids_remove_put_to_this_intersection
                       + vehicle_ids_remove_put_to_other_intersectionflow]
        for k in range(len(vehicles_put_to_this_intersection)):
            delta_i = VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT[str(None)] - VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT[str(direction)]
            new_i = i + delta_i
            new_flow[new_i].append(vehicles_put_to_this_intersection[k])
        for k in range(len(vehicles_put_to_other_intersectionflow)):
            toward_intersection_index = vehicles_put_to_other_intersectionflow[k].calc_toward_intersection_index()
            direction = vehicles_put_to_other_intersectionflow[k].direction
            if toward_intersection_index is None:
                new_index = -1
            else:
                new_index = 5 * toward_intersection_index + VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT[str(direction)]
            new_flow[new_index].append(vehicles_put_to_other_intersectionflow[k])
    for i in range(len(vehicles)):
        intersection_index = vehicles[i].calc_toward_intersection_index()
        direction = vehicles[i].direction
        # calc index_in_new_flow
        index = 5 * intersection_index + VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT[str(direction)]
        same_point = False
        for j in range(len(new_flow[index])):
            if new_flow[index][j].point.equal(vehicles[i].point):
                same_point = True
                break
        if not same_point:
            v = copy.deepcopy(vehicles[i])
            new_flow[index].append(v)
    return new_flow

# input: make sure that intersection_index is not None
# queue: queue of all vehicles in front of intersection, including moving vehicles and stationary vehicles
# stationary_queue: queue of stationary vehicles, while moving vehicles are excluded.
def calc_queue_and_stationary_queue_for_sorted_vehicles_of_same_direction_and_intersection(vehs: List[Vehicle]) -> List[int]:
    if len(vehs) == 0:
        return [0, 0]
    if not is_point_intersection(vehs[0].nextpoint):
        return [0, 0]
    queue = 1
    if vehs[0].should_move:
        stationary_queue = 0
    else:
        stationary_queue = 1
    for i in range(1, len(vehs)):
        dist = int(abs(vehs[i].point.x - vehs[i - 1].point.x) + abs(vehs[i].point.y - vehs[i - 1].point.y))
        if dist == BLOCK_DIST:
            queue += 1
            if stationary_queue != 0:
                stationary_queue += 1
        else:
            break
    return [queue, stationary_queue]

# executed after updating should_move and movement
def calc_queue_and_stationary_queue_in_a_step(flow_in_a_step: List[List[Vehicle]]) -> List[List[int]]:
    queue = [0] * (5 * len(INTERSECTIONS))
    stationary_queue = [0] * (5 * len(INTERSECTIONS))
    for i in range(len(flow_in_a_step) - 1):
        direction = INDEX_VEHICLE_MOVEMENT_DIRECTION_DICT[str(i % 5)]
        if direction is None:  # vehicle is in an intersection
            continue
        queue[i], stationary_queue[i] = \
            calc_queue_and_stationary_queue_for_sorted_vehicles_of_same_direction_and_intersection(
                vehs=flow_in_a_step[i])
    transferred_queue = [queue[i] for i in range(len(queue)) if i % 5 != 4]
    transferred_stationary_queue = [stationary_queue[i] for i in range(len(stationary_queue)) if i % 5 != 4]
    return transferred_queue, transferred_stationary_queue

# output: flow List[List[List[Vehicle]]]
# e.g., flow: List[List[List[Vehicle]]] = [flow0_intersection0_toeast, flow0_intersection0_towest, flow0_intersection0_tonorth, flow0_intersection0_tosouth, flow0_intersection0_in, ..., flow0_intersectionNONE_toeast, flow0_intersectionNONE_towest, flow0_intersectionNONE_tonorth, flow0_intersectionNONE_tosouth, flow0_intersectionNONE_in].
# flow0_intersection0_toeast: List[Vehicle] = [vehicle0, vehicle1, ...]
# output: traffic_light_states List[int]
# stationary_queues are calculated after updating should_move and movement
# queues are calculated after updating should_move and movement
# output flow at a step: size = 5 * len(INTERSECTIONS) + 1
# output queues at a step: size = 4 * len(INTERSECTIONS)
# output stationary_queues at a step: size = 4 * len(INTERSECTIONS)
def calc_vehicle_flow(start_step, end_step) -> List[Union[List[List[Vehicle]], List[int]]]:
    traffic_light_states = calc_traffic_light_states_in_intersections(start_step, end_step, len(INTERSECTIONS))
    flow = []  # [i] denotes the flow of the ith step,
    queues = []  # [i] denotes the flow of the ith step,
    stationary_queues = []  # [i] denotes the flow of the ith step,
    vs = calc_appeared_vehicles(start_step=start_step, end_step=end_step)
    for step in range(start_step, end_step + 1):
        traffic_light_states_this_step = traffic_light_states[step]
        appearing_vehicles = []
        for i in range(len(vs)):
            if vs[i].appearing_step >= step + 1:
                break
            if vs[i].appearing_step == step:
                vs[i].will_move = True
                v = copy.deepcopy(vs[i])
                appearing_vehicles.append(v)
        flow_pre_step = [[] for _ in range(5 * len(INTERSECTIONS) + 1)] if step == start_step else flow[-1]
        flow_this_step = calc_new_flow_by_adding_vehicles_in_a_step(vehicles=appearing_vehicles,
                                                                    cur_flow=flow_pre_step,
                                                                    traffic_light_states=traffic_light_states_this_step)
        queue, stationary_queue = calc_queue_and_stationary_queue_in_a_step(flow_in_a_step=flow_this_step)
        flow.append(flow_this_step)
        queues.append(queue)
        stationary_queues.append(stationary_queue)
        for i in range(len(flow_this_step)):
            for j in range(len(flow_this_step[i])):
                if_print = False
                if if_print:
                    print(f'step: {step}, directions: {flow_this_step[i][j].directions}, should_move: {flow_this_step[i][j].should_move}, traffic_light_states_this_step: {traffic_light_states_this_step}, point: ({flow_this_step[i][j].point.x}, {flow_this_step[i][j].point.y}), nextpoint: ({flow_this_step[i][j].nextpoint.x}, {flow_this_step[i][j].nextpoint.y}), next2point: ({flow_this_step[i][j].next2point.x}, {flow_this_step[i][j].next2point.y}),')
    return flow, queues, stationary_queues, traffic_light_states

def transfer_flow_to_list_of_vehicles(flow: List[List[List[Vehicle]]]) -> List[List[Vehicle]]:
    vehicles = []
    for i in range(len(flow)):
        vs = []
        for j in range(len(flow[i])):
            for k in range(len(flow[i][j])):
                v = copy.deepcopy(flow[i][j][k])
                vs.append(v)
        vehicles.append(vs)
    return vehicles

# calc the enter points
def calc_enter_points(type: str) -> List[Point]:
    assert type in ['eastwest', 'northsouth']
    enter_points = []
    if ROAD_MODEL == 0:
        if type == 'eastwest':
            p = Point(-BLOCK_DIST, 0)
        elif type == 'northsouth':
            p = Point(0, BLOCK_DIST)
        enter_points.append(p)
    elif ROAD_MODEL == 1:
        if type == 'eastwest':
            p1 = Point(-4.5 * BLOCK_DIST, -0.5 * BLOCK_DIST)
            p2 = Point(4.5 * BLOCK_DIST, 0.5 * BLOCK_DIST)
        elif type == 'northsouth':
            p1 = Point(-0.5 * BLOCK_DIST, 4.5 * BLOCK_DIST)
            p2 = Point(0.5 * BLOCK_DIST, -4.5 * BLOCK_DIST)
        enter_points.append(p1)
        enter_points.append(p2)
    elif ROAD_MODEL == 2:
        if type == 'eastwest':
            # east
            for y_ in Y_OF_INTERSECTIONS:
                y = y_ - 0.5 * BLOCK_DIST
                p = Point(X_MIN, y)
                enter_points.append(p)
            # west
            for y_ in Y_OF_INTERSECTIONS:
                y = y_ + 0.5 * BLOCK_DIST
                p = Point(X_MAX, y)
                enter_points.append(p)
        elif type == 'northsouth':
            # north
            for x_ in X_OF_INTERSECTIONS:
                x = x_ - 0.5 * BLOCK_DIST
                p = Point(x, Y_MAX)
                enter_points.append(p)
            # south
            for x_ in X_OF_INTERSECTIONS:
                x = x_ + 0.5 * BLOCK_DIST
                p = Point(x, Y_MIN)
                enter_points.append(p)
    return enter_points

def calc_an_appeared_vehicle(args):
    vehicle = None
    seed1 = args[0]
    seed2 = args[1]
    enter_point = args[2]
    eastwest_enter_points = args[3]
    id_ = args[4]
    appearing_step = args[5]

    random.seed(seed1)
    r1 = random.random()
    binomial_param = BINOMIAL_PARAM_EAST_WEST if enter_point in eastwest_enter_points else BINOMIAL_PARAM_NORTH_SOUTH
    if r1 < random.uniform(0, binomial_param):
        random.seed(seed2)
        r2 = random.random()
        action_in_intersection = ''
        if r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY:
            action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
        elif r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY:
            action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_LEFT
        elif r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_RIGHT_PROBABILITY:
            action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_RIGHT
        vehicle = Vehicle(id_=id_,
                          appearing_step=appearing_step,
                          startpoint=enter_point,
                          action_in_intersection=action_in_intersection)
        vehicle.update_points()
    return vehicle


# output: -> List[Vehicle], sorted by appearing_step
def calc_appeared_vehicles(start_step: int, end_step: int) -> List[Vehicle]:
    vehicles = []
    id_ = 0
    seed1 = 50
    seed2 = 100
    eastwest_enter_points = calc_enter_points(type='eastwest')
    northsouth_enter_points = calc_enter_points(type='northsouth')
    for appearing_step in range(start_step, end_step + 1):
        enter_points = eastwest_enter_points + northsouth_enter_points
        if PARALLEL == 1:
            args = []
            for enter_point in enter_points:
                arg = (seed1, seed2, enter_point, eastwest_enter_points, id_, appearing_step)
                seed1 += 1
                seed2 += 1
                id_ += 1
                args.append(arg)
            with Pool(NUM_CORES) as p:
                outputs = p.map(calc_an_appeared_vehicle, args)
            vehicles = []
            for i in range(len(outputs)):
                if outputs[i] is not None:
                    vehicles.append(outputs[i])
        else:
            for enter_point in enter_points:
                random.seed(seed1)
                r1 = random.random()
                seed1 += 1
                binomial_param = BINOMIAL_PARAM_EAST_WEST if enter_point in eastwest_enter_points else BINOMIAL_PARAM_NORTH_SOUTH
                if r1 < random.uniform(0, binomial_param):
                    random.seed(seed2)
                    r2 = random.random()
                    seed2 += 1
                    action_in_intersection = ''
                    if r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY:
                        action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
                    elif r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY:
                        action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_LEFT
                    elif r2 < VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_LEFT_PROBABILITY + VEHICLE_ACTION_IN_INTERSECTION_RIGHT_PROBABILITY:
                        action_in_intersection = VEHICLE_ACTION_IN_INTERSECTION_RIGHT
                    id_ += 1
                    vehicle = Vehicle(id_=id_,
                                      appearing_step=appearing_step,
                                      startpoint=enter_point,
                                      action_in_intersection=action_in_intersection)
                    vehicle.update_points()
                    vehicles.append(vehicle)
    vehicles = sorted(vehicles, key=lambda a: a.appearing_step, reverse=False)
    return vehicles


    # def calc_vehicles_between_self_intersection(self, vehicles: list):
    #     from fun import is_on_segment
    #     vehicles_between = []
    #     for v in vehicles:
    #         on = is_on_segment(v.point, self._point, self._intersections_will_pass[0])
    #         if on:
    #             vehicles_between.append(v)
    #     return vehicles_between


# if a vehicle is in intersections, it is put to a list
# in each list, sort vehicles
# calc queues in all intersections
def split_vehicles_and_calc_new_queues(vehicles: List[Vehicle], traffic_light_states: List[int]) -> List[Union[List[Vehicle], int]]:
    # here, stop is included.
    # x or y increases from start to stop or decreases from stop to start
    def calc_vehicels_satisfying_xy(vehs: List[Vehicle],
                                    fixed_x: Optional[int],
                                    fixed_y: Optional[int],
                                    start: int,
                                    stop: int,
                                    direction: str) -> List[Vehicle]:
        vehis = []
        if direction == VEHICLE_MOVEMENT_DIRECTION_TO_EAST:
            # x from max to min
            start_ = int(max(start, stop))
            stop_ = int(min(start, stop) - 1)
            step_ = -BLOCK_DIST
        elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_WEST:
            # x from min to max
            start_ = int(min(start, stop))
            stop_ = int(max(start, stop) + 1)
            step_ = BLOCK_DIST
        elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_NORTH:
            # y from max to min
            start_ = int(max(start, stop))
            stop_ = int(min(start, stop) - 1)
            step_ = -BLOCK_DIST
        elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH:
            # y from min to max
            start_ = int(min(start, stop))
            stop_ = int(max(start, stop) + 1)
            step_ = BLOCK_DIST
        else:
            raise ValueError('wrong direction')
        for value in range(start_, stop_, step_):
            for i in range(len(vehs)):
                if fixed_x is not None:
                    if vehs[i].point.x == fixed_x and vehs[i].point.y == value:
                        vehis.append(vehs[i])
                else:
                    if vehs[i].point.x == value and vehs[i].point.y == fixed_y:
                        vehis.append(vehs[i])
        return vehis
    # vehs are sorted, and in the same direction and intersection
    # if the first vehicle moves to an intersection, it will be put to the second list, since its direction may be changed
    def move_sorted_vehicles_of_same_direction_and_intersection(vehs: List[Vehicle],
                                                                traffic_light_states) \
            -> (List[Vehicle], List[Vehicle]):
        vehis = copy.deepcopy(vehs)
        if len(vehis) == 0:
            return []
        direction = vehis[0].direction
        intersection_index = vehis[0].calc_toward_intersection_index()
        if intersection_index is None:
            return []
        traffic_light_state = traffic_light_states[intersection_index]
        all_move = False
        if not is_point_intersection(vehis[0].nextpoint):
            all_move = True
        if direction in [VEHICLE_MOVEMENT_DIRECTION_TO_EAST, VEHICLE_MOVEMENT_DIRECTION_TO_WEST] and traffic_light_state == TRAFFIC_LIGHT_EASTWEST_PASS:
            all_move = True
        if direction in [VEHICLE_MOVEMENT_DIRECTION_TO_NORTH, VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH] and traffic_light_state == TRAFFIC_LIGHT_NORTHSOUTH_PASS:
            all_move = True
        for i in range(len(vehis)):
            if all_move:
                vehis[i].move()
            elif i != 0 and not vehis[i].point.equal(vehis[i - 1].point):
                vehis[i].move()
        if is_point_intersection(vehis[0].point):
            return vehis[1:], [vehis[0]]
        else:
            return vehis, []



    def calc_queue_for_sorted_vehicles_of_same_direction_and_intersection(vehs: List[Vehicle]) -> int:
        if len(vehs) == 0:
            return 0
        intersection_index = vehs[0].calc_toward_intersection_index()
        if intersection_index is None:
            return 0
        if not is_point_intersection(vehs[0].nextpoint):
            return 0
        queue = 1
        for i in range(1, len(vehs)):
            dist = int(abs(vehs[i].point.x - vehs[i - 1].point.x) + abs(vehs[i].point.y - vehs[i - 1].point.y))
            if dist == BLOCK_DIST:
                queue += 1
            else:
                break
        return queue

    def update_queues(vs_to_direction: List[Vehicle], queues: List[int]):
        vs_to_direction_moved = move_sorted_vehicles_of_same_direction_and_intersection(vs_to_direction)
        queue = calc_queue_for_sorted_vehicles_of_same_direction_and_intersection(vs_to_direction_moved)
        if len(vs_to_direction_moved) >= 1:
            intersection_index = vs_to_direction_moved[0].calc_toward_intersection_index()
            if intersection_index is not None:
                queues[intersection_index] = queue

    vs = []  # return
    queues = [0] * len(INTERSECTIONS)  # return
    for i in range(len(vehicles)):
        if is_point_intersection(vehicles[i].point):
            vs.append([vehicles[i]])
    if ROAD_MODEL == 0:
        # for i in range(len(vehicles)):
        #     if not is_point_intersection(vehicles[i].point):
        #         vs.append([vehicles[i]])
        vs_to_east1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=0,
                                                  start=X_MIN,
                                                  stop=X_MIN,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
        vs_to_east2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=0,
                                                  start=X_MAX,
                                                  stop=X_MAX,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
        vs_to_south1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=-0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=Y_MAX,
                                                   stop=Y_MAX,
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
        vs_to_south2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=-0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=Y_MIN,
                                                   stop=Y_MIN,
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
        vs.append(vs_to_east1)
        vs.append(vs_to_east2)
        vs.append(vs_to_south1)
        vs.append(vs_to_south2)
        update_queues(vs_to_east1, queues)
        update_queues(vs_to_east2, queues)
        update_queues(vs_to_south1, queues)
        update_queues(vs_to_south2, queues)
    elif ROAD_MODEL == 1:
        vs_to_east1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=-0.5 * BLOCK_DIST,
                                                  start=int(-1.5 * BLOCK_DIST),
                                                  stop=X_MIN,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
        vs_to_east2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=-0.5 * BLOCK_DIST,
                                                  start=int(1.5 * BLOCK_DIST),
                                                  stop=X_MAX,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
        vs_to_west1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=0.5 * BLOCK_DIST,
                                                  start=int(1.5 * BLOCK_DIST),
                                                  stop=X_MAX,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
        vs_to_west2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                  fixed_x=None,
                                                  fixed_y=0.5 * BLOCK_DIST,
                                                  start=int(-1.5 * BLOCK_DIST),
                                                  stop=X_MIN,
                                                  direction=VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
        vs_to_north1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=Y_MIN,
                                                   stop=int(-1.5 * BLOCK_DIST),
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
        vs_to_north2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=int(1.5 * BLOCK_DIST),
                                                   stop=Y_MAX,
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
        vs_to_south1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=-0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=Y_MAX,
                                                   stop=int(1.5 * BLOCK_DIST),
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
        vs_to_south2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                   fixed_x=-0.5 * BLOCK_DIST,
                                                   fixed_y=None,
                                                   start=int(-1.5 * BLOCK_DIST),
                                                   stop=Y_MIN,
                                                   direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
        vs.append(vs_to_east1)
        vs.append(vs_to_east2)
        vs.append(vs_to_west1)
        vs.append(vs_to_west2)
        vs.append(vs_to_north1)
        vs.append(vs_to_north2)
        vs.append(vs_to_south1)
        vs.append(vs_to_south2)
        update_queues(vs_to_east1, queues)
        update_queues(vs_to_west1, queues)
        update_queues(vs_to_north1, queues)
        update_queues(vs_to_south1, queues)
    elif ROAD_MODEL == 2:
        # fix y, calc east west
        for y_ in Y_OF_INTERSECTIONS:
            # to east
            y = y_ - 0.5 * BLOCK_DIST
            vs_to_east1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                      fixed_x=None,
                                                      fixed_y=y,
                                                      start=X_MIN,
                                                      stop=int(X_OF_INTERSECTIONS[0] - 1.5 * BLOCK_DIST),
                                                      direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
            if len(vs_to_east1) > 0:
                vs.append(vs_to_east1)
                update_queues(vs_to_east1, queues)
            for i in range(len(X_OF_INTERSECTIONS) - 1):
                vs_to_east2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                          fixed_x=None,
                                                          fixed_y=y,
                                                          start=int(X_OF_INTERSECTIONS[i] + 1.5 * BLOCK_DIST),
                                                          stop=int(X_OF_INTERSECTIONS[i + 1] - 1.5 * BLOCK_DIST),
                                                          direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
                if len(vs_to_east2) > 0:
                    vs.append(vs_to_east2)
                    update_queues(vs_to_east2, queues)
            vs_to_east3 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                      fixed_x=None,
                                                      fixed_y=y,
                                                      start=int(X_OF_INTERSECTIONS[-1] + 1.5 * BLOCK_DIST),
                                                      stop=X_MAX,
                                                      direction=VEHICLE_MOVEMENT_DIRECTION_TO_EAST)
            if len(vs_to_east3) > 0:
                vs.append(vs_to_east3)
            # to west
            y = y_ + 0.5 * BLOCK_DIST
            vs_to_west1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                      fixed_x=None,
                                                      fixed_y=y,
                                                      start=int(X_OF_INTERSECTIONS[-1] + 1.5 * BLOCK_DIST),
                                                      stop=X_MAX,
                                                      direction=VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
            if len(vs_to_west1) > 0:
                vs.append(vs_to_west1)
                update_queues(vs_to_west1, queues)
            for i in range(len(X_OF_INTERSECTIONS) - 1):
                vs_to_west2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                          fixed_x=None,
                                                          fixed_y=y,
                                                          start=int(X_OF_INTERSECTIONS[i] + 1.5 * BLOCK_DIST),
                                                          stop=int(X_OF_INTERSECTIONS[i + 1] - 1.5 * BLOCK_DIST),
                                                          direction=VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
                if len(vs_to_west2) > 0:
                    vs.append(vs_to_west2)
                    update_queues(vs_to_west2, queues)
            vs_to_west3 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                      fixed_x=None,
                                                      fixed_y=y,
                                                      start=X_MIN,
                                                      stop=int(X_OF_INTERSECTIONS[0] - 1.5 * BLOCK_DIST),
                                                      direction=VEHICLE_MOVEMENT_DIRECTION_TO_WEST)
            if len(vs_to_west3) > 0:
                vs.append(vs_to_west3)
        # fix x, calc north south
        for x_ in X_OF_INTERSECTIONS:
            # to north
            x = x_ + 0.5 * BLOCK_DIST
            vs_to_north1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                       fixed_x=x,
                                                       fixed_y=None,
                                                       start=Y_MIN,
                                                       stop=int(Y_OF_INTERSECTIONS[0] - 1.5 * BLOCK_DIST),
                                                       direction=VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
            if len(vs_to_north1) > 0:
                vs.append(vs_to_north1)
                update_queues(vs_to_north1, queues)
            for i in range(len(Y_OF_INTERSECTIONS) - 1):
                vs_to_north2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                           fixed_x=x,
                                                           fixed_y=None,
                                                           start=int(Y_OF_INTERSECTIONS[i] + 1.5 * BLOCK_DIST),
                                                           stop=int(Y_OF_INTERSECTIONS[i + 1] - 1.5 * BLOCK_DIST),
                                                           direction=VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
                if len(vs_to_north2) > 0:
                    vs.append(vs_to_north2)
                    update_queues(vs_to_north2, queues)
            vs_to_north3 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                       fixed_x=x,
                                                       fixed_y=None,
                                                       start=int(Y_OF_INTERSECTIONS[-1] + 1.5 * BLOCK_DIST),
                                                       stop=Y_MAX,
                                                       direction=VEHICLE_MOVEMENT_DIRECTION_TO_NORTH)
            if len(vs_to_north3) > 0:
                vs.append(vs_to_north3)

            # to south
            x = x_ - 0.5 * BLOCK_DIST
            vs_to_south1 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                       fixed_x=x,
                                                       fixed_y=None,
                                                       start=int(Y_OF_INTERSECTIONS[-1] + 1.5 * BLOCK_DIST),
                                                       stop=Y_MAX,
                                                       direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
            if len(vs_to_south1) > 0:
                vs.append(vs_to_south1)
                update_queues(vs_to_south1, queues)
            for i in range(len(Y_OF_INTERSECTIONS) - 1):
                # vs_to_south2 = []
                # y1 = int(Y_OF_INTERSECTIONS[i] + 1.5 * BLOCK_DIST)
                # y2 = int(Y_OF_INTERSECTIONS[i + 1] - 1.5 * BLOCK_DIST)
                # for y in range(y1, y2 + 1, BLOCK_DIST):
                #     for v in vehicles:
                #         if v.point.y == y and v.point.x == x:
                #             vs_to_south2.append(v)
                vs_to_south2 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                           fixed_x=x,
                                                           fixed_y=None,
                                                           start=int(Y_OF_INTERSECTIONS[i] + 1.5 * BLOCK_DIST),
                                                           stop=int(Y_OF_INTERSECTIONS[i + 1] - 1.5 * BLOCK_DIST),
                                                           direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
                if len(vs_to_south2) > 0:
                    vs.append(vs_to_south2)
                    update_queues(vs_to_south2, queues)
            vs_to_south3 = calc_vehicels_satisfying_xy(vehs=vehicles,
                                                       fixed_x=x,
                                                       fixed_y=None,
                                                       start=Y_MIN,
                                                       stop=int(Y_OF_INTERSECTIONS[0] - 1.5 * BLOCK_DIST),
                                                       direction=VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH)
            if len(vs_to_south3) > 0:
                vs.append(vs_to_south3)
    return vs

def calc_index_of_intersections(p: Point):
    index = None
    for i in range(len(INTERSECTIONS)):
        if p.equal(INTERSECTIONS[i]):
            index = i
            break
    return index

def is_point_intersection(point: Point):
    return point.x in X_OF_INTERSECTIONS and point.y in Y_OF_INTERSECTIONS

# update should_move for the vehicles, all vehicles move
# two processes: split vehicles, and update should_move, move
# return queues and stationary queues after movement
def update_should_move_and_calc_new_queues(vehicles: List[Vehicle], traffic_light_states: List[int]) -> List[int]:
    ids_should_move = []
    vs_matrix, new_queues = split_vehicles_and_calc_new_queues(vehicles, traffic_light_states)
    for vs in vs_matrix:
        all_back_should_move = False  # all the back vehicles should move
        for i in range(len(vs)):
            if all_back_should_move:
                ids_should_move.append(vs[i].id_)
            elif is_point_intersection(vs[i].point):
                ids_should_move.append(vs[i].id_)
            else:
                if i == 0:
                    if is_point_intersection(vs[i].nextpoint):
                        index_of_intersections = calc_index_of_intersections(vs[i].nextpoint)
                        traffic_light_state = traffic_light_states[index_of_intersections]
                        if vs[i].direction in [VEHICLE_MOVEMENT_DIRECTION_TO_EAST, VEHICLE_MOVEMENT_DIRECTION_TO_WEST]:
                            if traffic_light_state == TRAFFIC_LIGHT_EASTWEST_PASS:
                                ids_should_move.append(vs[i].id_)
                                all_back_should_move = True
                        elif vs[i].direction in [VEHICLE_MOVEMENT_DIRECTION_TO_NORTH, VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH]:
                            if traffic_light_state == TRAFFIC_LIGHT_NORTHSOUTH_PASS:
                                ids_should_move.append(vs[i].id_)
                                all_back_should_move = True
                    else:
                        ids_should_move.append(vs[i].id_)
                        all_back_should_move = True
                else:
                    delta_x = abs(vs[i].point.x - vs[i - 1].point.x)
                    delta_y = abs(vs[i].point.y - vs[i - 1].point.y)
                    assert delta_x == 0 or delta_y == 0
                    if delta_x + delta_y >= 2 * BLOCK_DIST:
                        ids_should_move.append(vs[i].id_)
                        all_back_should_move = True

    for i in range(len(vehicles)):
        if vehicles[i].id_ in ids_should_move:
            vehicles[i].should_move = True
            vehicles[i].move()
        else:
            vehicles[i].should_move = False
    return new_queues



if __name__ == '__main__':
    # traffic_light_states = calc_traffic_light_states(START_STEP, END_STEP)
    print('\n\n')
    start_time = time.time()
    flow, queues, stationary_queues, traffic_light_states = calc_vehicle_flow(start_step=START_STEP, end_step=END_STEP)
    duration = time.time() - start_time
    print('\n')
    print(f'PARALLEL: {PARALLEL}')
    print(f'duration: {duration}')
    print('finish calc_vehicle_flow')
    print('\n\n')