import math

import config
from typing import List


from config import VEHICLE_ACTION_IN_INTERSECTION_STRAIGHT
from config import VEHICLE_ACTION_IN_INTERSECTION_LEFT
from config import VEHICLE_ACTION_IN_INTERSECTION_RIGHT
from config import ROAD_MODEL
from config import SIGMA_SQUARE
from config import UAV_VERTICAL_MOVE_ACTION_DOWNWARD
from config import UAV_VERTICAL_MOVE_ACTION_STAY
from config import UAV_VERTICAL_MOVE_ACTION_UPWARD
from config import DIST_UAV_MOVE_DOWNWARD_UPWARD
from config import UAV_MIN_HEIGHT
from config import UAV_MAX_HEIGHT
from config import INTERSECTIONS
from config import ALPHA1
from config import ALPHA2
from config import BETA1
from config import BETA2
from config import LOS
from config import NLOS
from config import Point
from config import BLOCKS
from config import BANDWIDTH_OF_ONE_CHANNEL
from config import BLOCKS_FOR_UAV
# from fun import calc_point
# from fun import calc_horizontal_dist_between_blocks
from vehicle import Vehicle
class Uav:
    def _init_(self, point: Point, height: float, energy_ratio: float):
        self._point = point
        self._block = BLOCKS_FOR_UAV.index(self.point)
        self._height = height
        self._energy_ratio = energy_ratio

    @property
    def point(self):
        return self._point

    @point.setter
    def point(self, point: Point):
        self._point = point

    @property
    def block_id(self):
        return BLOCKS_FOR_UAV.index(self.point)

    # @block_id.setter
    # def block(self, block_id: int):
    #     self._block_id = block_id

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, height: int):
        self._height = height

    @property
    def energy_ratio(self):
        return self._energy_ratio

    @energy_ratio.setter
    def energy(self, energy_ratio: int):
        self._energy_ratio = energy_ratio

    # consumption: consumed energy
    def update_energy_ratio_by_energy_consumption_ratio(self, consumption_ratio):
        self.energy_ratio -= consumption_ratio

# if horizontal_move_action is not allowable, its default value is 0, i.e., stay at the cur point
    def calc_new_point(self, horizontal_move_action: int) -> Point:
        new_point = self.point
        if horizontal_move_action == 0:
            new_point = self.point
        elif self.point.equal(INTERSECTIONS[0]):
            if horizontal_move_action in [1, 2, 3, 4]:
                new_point = BLOCKS_FOR_UAV[horizontal_move_action]
            elif ROAD_MODEL == 1 and horizontal_move_action in [5, 6, 7, 8]:
                new_point = BLOCKS_FOR_UAV[horizontal_move_action - 3]
            # stay at the cur point
            else:
                new_point = self.point
        # anti-clockwise
        elif (ROAD_MODEL == 0 and horizontal_move_action == 5) or (ROAD_MODEL == 1 and horizontal_move_action == 9):
            if ROAD_MODEL == 0:
                new_point = BLOCKS_FOR_UAV[(self.block_id + 1) % 4]
            else:
                new_point = BLOCKS_FOR_UAV[(self.block_id + 1) % 8]
        # clockwise
        elif (ROAD_MODEL == 0 and horizontal_move_action == 7) or (ROAD_MODEL == 1 and horizontal_move_action == 11):
            if ROAD_MODEL == 0:
                new_point = BLOCKS_FOR_UAV[(self.block_id - 1) % 4]
            else:
                new_point = BLOCKS_FOR_UAV[(self.block_id - 1) % 8]
        # move to the intersection
        elif (ROAD_MODEL == 0 and horizontal_move_action == 6) or (ROAD_MODEL == 1 and horizontal_move_action == 10):
            new_point = INTERSECTIONS[0]
        # stay at the cur point
        else:
            new_point = self.point
        return new_point

    # assume the init energy is 1, return the energy_consumption_ratio
    def calc_energy_consumption_ratio_by_vertical_move_action(self, vertical_move_action: int):
        energy_consumption_ratio = -1
        if vertical_move_action == UAV_VERTICAL_MOVE_ACTION_DOWNWARD:
            energy_consumption_ratio = 6 / (27 * 60)
        elif vertical_move_action == UAV_VERTICAL_MOVE_ACTION_STAY:
            energy_consumption_ratio = 6 / (21 * 60)
        elif vertical_move_action == UAV_VERTICAL_MOVE_ACTION_UPWARD:
            energy_consumption_ratio = 6 / (17 * 60)
        else:
            raise ValueError("wrong vertical_move_action.")
        return energy_consumption_ratio


    def calc_new_height(self, vertical_move_action: int):
        new_height = self.height
        if vertical_move_action == UAV_VERTICAL_MOVE_ACTION_DOWNWARD:
            new_height = max(self.height - DIST_UAV_MOVE_DOWNWARD_UPWARD, UAV_MIN_HEIGHT)
        elif vertical_move_action == UAV_VERTICAL_MOVE_ACTION_STAY:
            new_height = self.height
        elif vertical_move_action == UAV_VERTICAL_MOVE_ACTION_UPWARD:
            new_height = min(self.height + DIST_UAV_MOVE_DOWNWARD_UPWARD, UAV_MAX_HEIGHT)
        else:
            raise ValueError("wrong vertical_move_action.")
        return new_height

    def update_point_and_height_by_move_action(self, horizontal_move_action, vertical_move_action):
        new_point = self.calc_new_point(horizontal_move_action)
        new_height = self.calc_new_height(vertical_move_action)
        self.point = new_point
        self.height = new_height

    # return LOS or NLOS for the link between uav and one block (point)
    def calc_channel_state(self, vehicle_point: Point) -> int:
        horizontal_dist = Point.calc_euclidean_dist(self.point, vehicle_point)
        los_probability = 1 / (1 + ALPHA1 * math.exp(-ALPHA2 * (180 / math.pi * math.atan(self.height / horizontal_dist) - ALPHA1)))
        import random
        r = random.random()
        if r < los_probability:
            channel_state = LOS
        else:
            channel_state = NLOS
        return channel_state

    # return LOS or NLOS for the link between uav and all blocks
    def calc_channel_states(self) -> List[int]:
        channel_states = []
        for i in range(len(BLOCKS)):
            channel_state = self.calc_channel_state(BLOCKS[i])
            channel_states.append(channel_state)
        return channel_states


    def calc_channel_power_gain(self, vehicle_point: Point, channel_state: int) -> float:
        horizontal_dist = Point.calc_euclidean_dist(self.point, vehicle_point)
        dist = (horizontal_dist ** 2 + self.height ** 2) ** 0.5
        if channel_state == LOS:
            channel_power_gain = dist ** (-BETA1)
        else:
            channel_power_gain = BETA2 * dist ** (-BETA1)
        return channel_power_gain

    def calc_total_throughput(self, flow_this_step: List[List[Vehicle]], powers: List[float], channels: List[int]) -> float:
        total_throughput: float = 0.0
        for i in range(len(flow_this_step)):
            for j in range(len(flow_this_step[i])):
                vehicle = flow_this_step[i][j]
                channel_state = self.calc_channel_state(vehicle.point)
                channel_power_gain = self.calc_channel_power_gain(vehicle.point, channel_state)
                idx = BLOCKS.index(vehicle.point)
                power = powers[idx]
                channel = channels[idx]
                throughput = BANDWIDTH_OF_ONE_CHANNEL * channels[i] * math.log(2, 1 + power * channel_power_gain / BANDWIDTH_OF_ONE_CHANNEL / channel / SIGMA_SQUARE)
                total_throughput += throughput
        return total_throughput










