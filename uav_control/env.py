import copy
import math
import torch
import gym
import os
import time
import sys
from gym import spaces, logger
from gym.utils import seeding
from gym.spaces.space import Space
from gym.spaces.box import Box
from gym.spaces.discrete import Discrete
# from gym.spaces
from gym.spaces.multi_discrete import MultiDiscrete
from gym.spaces.multi_binary import MultiBinary
from gym.spaces.tuple import Tuple
from gym.spaces.dict import Dict
import numpy as np
from typing import List
from uav_control.uav import Uav
from config import VEHICLE_MOVEMENT_DIRECTION_TO_EAST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_WEST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
from config import VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
from config import START_TIMESLOT
from config import END_TIMESLOT
from config import LOS
from config import NLOS
from config import BLOCKS
from config import BLOCKS_FOR_UAV
from config import UAV_MIN_HEIGHT
from config import UAV_MAX_HEIGHT
from config import ROAD_MODEL
from config import INTERSECTIONS
from config import START_STEP
from config import END_STEP
from config import BLOCKS
from config import BLOCKS_FOR_UAV
from config import Point
from config import BLOCK_DIST
from config import VEHICLE_MOVEMENT_DIRECTION_INDEX_DICT
from config import INDEX_VEHICLE_MOVEMENT_DIRECTION_DICT
from vehicle import calc_appeared_vehicles
from vehicle import Vehicle
from vehicle import filter_vehicles
from vehicle import update_should_move_and_calc_new_queues
from vehicle import is_point_intersection
from vehicle import calc_new_flow_by_adding_vehicles_in_a_step
from vehicle import calc_queue_and_stationary_queue_in_a_step
# from traffic_light_control.env import
# from elegantrl.agents import AgentDQN
# from elegantrl.agents import AgentPPO
# from elegantrl.run import train_and_evaluate
# from elegantrl.run import Arguments
class UavControlEnv(gym.Env):
    # def __init__(
    #         self,
    #         gamma=0.99,
    #         reward_scaling=2 ** -11,
    # ):


    # state: traffic_light, x, y, z, n, h
    def calc_state_dim(self) -> int:
        block_num = len(BLOCKS)
        state_dim = 4 + block_num + block_num
        return state_dim

    def calc_action_dim(self) -> int:
        block_num = len(BLOCKS)
        action_dim = 2 + block_num + block_num
        return action_dim

    def __init__(self, gamma: float = 0.99):
        assert ROAD_MODEL in [0, 1]
        # self.df_pwd = './China_A_shares.pandas.dataframe'
        # self.npz_pwd = './China_A_shares.numpy.npz'
        self.traffic_light_states: List[int] = None

        # 1: vehicle exist, 0: empty
        self.vehicle_indicators_in_blocks: List[int] = None

        uav_point = INTERSECTIONS[0]
        uav_height = UAV_MIN_HEIGHT
        uav_energy_ratio = 1
        self.uav: Uav = Uav(uav_point, uav_height, uav_energy_ratio)

        self.channel_states: List[int] = [LOS] * len(BLOCKS)

        self.step_: int = START_STEP  # START_STEP <= self.step_ <= END_STEP
        self.gamma: float = gamma
        self.appeared_vehicles: List[Vehicle] = None
        self.flow: List[List[Vehicle]] = []  # [[flow_step0], [flow_step1], ...]

        # reset()
        self.rewards = None
        self.cumulative_returns = 0

        # environment information
        self.env_name = 'UavControl'
        self.if_discrete = True if ROAD_MODEL == 0 else False
        self.state_dim: int = self.calc_state_dim()
        self.action_dim: int = self.calc_action_dim()
        self.max_step: int = END_STEP
        self.target_return = +np.inf

    def get_state(self):
        state = np.hstack([self.traffic_light_states, [self.uav.point.x, self.uav.point.y, self.uav.height], self.vehicle_indicators_in_blocks, self.channel_states])
        return state

    def calc_vehicle_indicators_in_blocks(self, flow_this_step: List[List[Vehicle]]):
        vehicle_indicators = [0] * len(BLOCKS)
        for i in range(flow_this_step):
            for j in range(flow_this_step[i]):
                idx = BLOCKS.index(flow_this_step[i][j])
                vehicle_indicators[idx] = 1
        return vehicle_indicators

    def calc_appeared_vehicles_this_step(self):
        appeared_vehicles_this_step = []
        index_of_appeared_vehicles = None
        for i in range(len(self.appeared_vehicles)):
            if self.appeared_vehicles[i].appearing_step >= self.step_ + 1:
                index_of_appeared_vehicles = i
                break
            if self.appeared_vehicles[i].appearing_step == self.step_:
                self.appeared_vehicles[i].will_move = True
                v = copy.deepcopy(self.appeared_vehicles[i])
                appeared_vehicles_this_step.append(v)
        self.appeared_vehicles = self.appeared_vehicles[index_of_appeared_vehicles:]
        return appeared_vehicles_this_step


    # state: light0, light1, ..., queue_{0, 0}, queue_{0, 1},queue_{0, 2},queue_{0, 3}, ...
    # four queues in one intersection: [to_EAST, to_WEST, to_NORTH, to_SOUTH]
    def reset(self):
        self.rewards = []
        self.cumulative_returns = 0
        self.traffic_light_states: List[int] = np.random.randint(low=0, high=4, size=len(INTERSECTIONS))
        # print("self.traffic_light_states: ", self.traffic_light_states)
        self.step_: int = START_STEP
        self.flow: List[List[Vehicle]] = []
        self.appeared_vehicles: List[Vehicle] = calc_appeared_vehicles(start_step=START_STEP,
                                                                       end_step=END_STEP)
        appeared_vehicles_this_step = self.calc_appeared_vehicles_this_step()
        flow_pre_step = [[] for _ in range(5 * len(INTERSECTIONS) + 1)]
        flow_this_step = calc_new_flow_by_adding_vehicles_in_a_step(vehicles=appeared_vehicles_this_step,
                                                                    cur_flow=flow_pre_step,
                                                                    traffic_light_states=self.traffic_light_states)
        self.flow.append(flow_this_step)
        self.vehicle_indicators_in_blocks = self.calc_vehicle_indicators_in_blocks(flow_this_step=flow_this_step)
        return self.get_state()

    def step(self, action: List[float]):
        print("self.step_: ", self.step_)
        print("len(action): ", len(action))
        print("len(INTERSECTIONS): ", len(INTERSECTIONS))
        print("len(BLOCKS): ", len(BLOCKS))
        print("len(BLOCKS_FOR_UAV): ", len(BLOCKS_FOR_UAV))
        uav_horizontal_move_action = action[0]
        uav_vertical_move_action = action[1]
        powers = action[2: 2 + len(BLOCKS)]
        channels = action[2 + len(BLOCKS):]

        for i in range(len(self.traffic_light_states)):
            self.traffic_light_states[i] = (self.traffic_light_states[i] + 1) % 4
        self.uav.point = self.uav.calc_new_point(uav_horizontal_move_action)
        self.uav.height = self.uav.calc_new_height(uav_vertical_move_action)

        self.step_: int = self.step_ + 1

        appeared_vehicles_this_step = self.calc_appeared_vehicles_this_step()
        flow_pre_step = self.flow[-1]
        flow_this_step = calc_new_flow_by_adding_vehicles_in_a_step(vehicles=appeared_vehicles_this_step,
                                                                    cur_flow=flow_pre_step,
                                                                    traffic_light_states=self.traffic_light_states)
        self.vehicle_indicators_in_blocks = self.calc_vehicle_indicators_in_blocks(flow_this_step)
        self.channel_states = self.uav.calc_channel_states()

        self.flow.append(flow_this_step)

        state = self.get_state()
        reward = self.uav.calc_total_throughput(flow_this_step, powers, channels)

        self.rewards.append(reward)
        done = self.step_ == self.max_step - 1
        if done:
            reward += 1 / (1 - self.gamma) * np.mean(self.rewards)
            self.cumulative_returns = sum(self.rewards)
        return state, reward, done, {}


    def seed(self, seed=None):
        pass



def check_env():
    env = UavControlEnv()
    evaluate_time = 4

    print()
    policy_name = 'random_action'
    state = env.reset()
    for _ in range(env.max_step * evaluate_time):
        action = np.random.randint(low=0, high=2, size=env.get)
        print("action in check_env", action)
        state, reward, done, _ = env.step(action)
        if done:
            print(f'cumulative_returns of {policy_name}: {env.cumulative_returns:9.2f}')
            state = env.reset()
            break

    policy_name = 'continuing'
    state = env.reset()
    for _ in range(env.max_step * evaluate_time):
        action = np.zeros(intersection_num)
        print("action in check_env", action)
        state, reward, done, _ = env.step(action)
        if done:
            print(f'cumulative_returns of {policy_name}: {env.cumulative_returns:9.2f}')
            state = env.reset()
            break

    policy_name = 'changing'
    state = env.reset()
    for _ in range(env.max_step * evaluate_time):
        action = np.ones(intersection_num)
        print("action in check_env", action)
        state, reward, done, _ = env.step(action)
        if done:
            print(f'cumulative_returns of {policy_name}: {env.cumulative_returns:9.2f}')
            state = env.reset()
            break

def get_gym_env_args(env, if_print) -> dict:  # [ElegantRL.2021.12.12]
    """
    Get a dict ``env_args`` about a standard OpenAI gym env information.

    :param env: a standard OpenAI gym env
    :param if_print: [bool] print the dict about env information.
    :return: env_args [dict]

    env_args = {
        'env_num': 1,               # [int] the environment number, 'env_num>1' in vectorized env
        'env_name': env_name,       # [str] the environment name, such as XxxXxx-v0
        'max_step': max_step,       # [int] the steps in an episode. (from env.reset to done).
        'state_dim': state_dim,     # [int] the dimension of state
        'action_dim': action_dim,   # [int] the dimension of action or the number of discrete action
        'if_discrete': if_discrete, # [bool] action space is discrete or continuous
    }
    """
    import gym

    env_num = getattr(env, 'env_num') if hasattr(env, 'env_num') else 1

    if {'unwrapped', 'observation_space', 'action_space', 'spec'}.issubset(dir(env)):  # isinstance(env, gym.Env):
        env_name = getattr(env, 'env_name', None)
        env_name = env.unwrapped.spec.id if env_name is None else env_name

        state_shape = env.observation_space.shape
        state_dim = state_shape[0] if len(state_shape) == 1 else state_shape  # sometimes state_dim is a list

        max_step = getattr(env, 'max_step', None)
        max_step_default = getattr(env, '_max_episode_steps', None)
        if max_step is None:
            max_step = max_step_default
        if max_step is None:
            max_step = 2 ** 10

        if_discrete = isinstance(env.action_space, gym.spaces.Discrete)
        if if_discrete:  # make sure it is discrete action space
            action_dim = env.action_space.n
        elif isinstance(env.action_space, gym.spaces.Box):  # make sure it is continuous action space
            action_dim = env.action_space.shape[0]
            if not any(env.action_space.high - 1):
                print('WARNING: env.action_space.high', env.action_space.high)
            if not any(env.action_space.low - 1):
                print('WARNING: env.action_space.low', env.action_space.low)
        else:
            raise RuntimeError('\n| Error in get_gym_env_info()'
                               '\n  Please set these value manually: if_discrete=bool, action_dim=int.'
                               '\n  And keep action_space in (-1, 1).')
    else:
        env_name = env.env_name
        max_step = env.max_step
        state_dim = env.state_dim
        action_dim = env.action_dim
        if_discrete = env.if_discrete

    env_args = {'env_num': env_num,
                'env_name': env_name,
                'max_step': max_step,
                'state_dim': state_dim,
                'action_dim': action_dim,
                'if_discrete': if_discrete, }
    if if_print:
        env_args_repr = repr(env_args)
        env_args_repr = env_args_repr.replace(',', f",\n   ")
        env_args_repr = env_args_repr.replace('{', "{\n    ")
        env_args_repr = env_args_repr.replace('}', ",\n}")
        print(f"env_args = {env_args_repr}")
    return env_args

class ReplayBufferList(list):  # for on-policy
    def __init__(self):
        list.__init__(self)

    def update_buffer(self, traj_list):
        cur_items = [map(list, zip(*traj_list))]
        self[:] = [torch.cat(item, dim=0) for item in cur_items]

        steps = self[1].shape[0]
        r_exp = self[1].mean().item()
        return steps, r_exp

def train_agent(args):
    torch.set_grad_enabled(False)
    args.init_before_training()
    gpu_id = args.learner_gpus

    '''init'''
    env = UavControlEnv()

    agent = args.agent_class(args.net_dim, args.state_dim, args.action_dim, gpu_id=gpu_id, args=args)
    agent.states = [env.reset(), ]

    buffer = ReplayBufferList()

    '''start training'''
    cwd = args.cwd
    break_step = args.break_step
    target_step = args.target_step
    del args

    start_time = time.time()
    total_step = 0
    save_gap = int(5e4)
    total_step_counter = -save_gap
    while True:
        trajectory = agent.explore_env(env, target_step)
        steps, r_exp = buffer.update_buffer((trajectory,))

        torch.set_grad_enabled(True)
        logging_tuple = agent.update_net(buffer)
        torch.set_grad_enabled(False)

        total_step += steps

        if total_step_counter + save_gap < total_step:
            total_step_counter = total_step
            print(
                f"Step:{total_step:8.2e}  "
                f"ExpR:{r_exp:8.2f}  "
                f"Returns:{env.cumulative_returns:8.2f}  "
                f"ObjC:{logging_tuple[0]:8.2f}  "
                f"ObjA:{logging_tuple[1]:8.2f}  "
            )
            save_path = f"{cwd}/actor_{total_step:014.0f}_{time.time() - start_time:08.0f}_{r_exp:08.2f}.pth"
            torch.save(agent.act.state_dict(), save_path)

        if (total_step > break_step) or os.path.exists(f"{cwd}/stop"):
            # stop training when reach `break_step` or `mkdir cwd/stop`
            break

    print(f'| UsedTime: {time.time() - start_time:.0f} | SavedDir: {cwd}')

def run():
    import sys
    gpu_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    env = UavControlEnv()
    env_func = UavControlEnv
    env_args = get_gym_env_args(env=env, if_print=False)

    # args = Arguments(AgentPPO, env_func=env_func, env_args=env_args)
    # args.target_step = args.max_step * 4
    # args.reward_scale = 2 ** -7
    # args.learning_rate = 2 ** -14
    # args.break_step = int(5e5)
    #
    # args.learner_gpus = gpu_id
    # args.random_seed += gpu_id + 1943
    # train_agent(args)


if __name__ == '__main__':
    check_env()
