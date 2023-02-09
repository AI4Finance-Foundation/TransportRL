import random
from typing import List

class TrafficLight:
    def __init__(self, state: int = 0):
        self._state = state
        # self._timeslot = timeslot

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state: int):
        self._state = state

    def act(self, action: int):
        if action == 0:
            return
        elif action == 1:
            self.state = (self.state + 1) % 4
        else:
            raise ValueError("wrong action.")


# # return a list, including start_step and end_step
# def calc_traffic_light_states_in_an_intersection(start_step: int,
#                               end_step: int,
#                               init_state: int = random.randint(0, 3)) -> List[int]:
#     traffic_light_states = []
#     traffic_light_state = init_state
#     traffic_light_states.append(traffic_light_state)
#     for timeslot in range(start_step, end_step + 1):
#         traffic_light_state = (traffic_light_state + 1) % 4
#         traffic_light_states.append(traffic_light_state)
#     return traffic_light_states[: -1]

# return a list, including start_step and end_step
def calc_traffic_light_states_in_intersections(start_step: int,
                                               end_step: int,
                                               intersections_num: int) -> List[int]:
    traffic_light_states = []
    for i in range(start_step, end_step + 1):
        light_states = []
        if i == start_step:
            for _ in range(intersections_num):
                init_state = random.randint(0, 3)
                light_states.append(init_state)
        else:
            for j in range(intersections_num):
                light_state = (traffic_light_states[-1][j] + 1) % 4
                light_states.append(light_state)
        traffic_light_states.append(light_states)
    return traffic_light_states
