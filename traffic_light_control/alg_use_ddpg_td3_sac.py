import sys
import gym
import numpy as np

from elegantrl.train.run import train_agent, train_agent_multiprocessing
from elegantrl.train.config import Config
from elegantrl.agents.AgentDDPG import AgentDDPG
from elegantrl.agents.AgentTD3 import AgentTD3
from elegantrl.agents.AgentSAC import AgentSAC


from config import INTERSECTIONS
from env import TrafficLightControlEnv
from config import ROAD_MODEL

def alg_continuous_action_off_policy(GPU_ID, DRL_ID):  # 2022.02.02
    assert ROAD_MODEL == 2
    agent_class = [AgentDDPG, AgentTD3, AgentSAC][DRL_ID]  # DRL algorithm name
    env_class = TrafficLightControlEnv
    env = TrafficLightControlEnv()
    env_args = {
        'env_name': 'TrafficLightControlEnv',
        'max_step': 2000,  # the max step number of an episode.
        'state_dim': env.calc_state_dim(),
        'action_dim': env.calc_action_dim(),
        'if_discrete': False
    }

    args = Config(agent_class, env_class, env_args)  
    args.break_step = int(8e4)  # break training if 'total_step > break_step'
    args.net_dims = (128, 64)  # the middle layer dimension of MultiLayer Perceptron
    args.gamma = 0.97  # discount factor of future rewards
    args.horizon_len = args.max_step * 2

    args.repeat_times = 1.0  # repeatedly update network using ReplayBuffer to keep critic's loss small
    args.learning_rate = 1e-4
    args.state_value_tau = 0.1  # the tau of normalize for value and state `std = (1-std)*std + tau*std`

    args.gpu_id = GPU_ID
    args.num_workers = 4
    if_single_process = True
    if if_single_process:
        train_agent(args)
    else:
        train_agent_multiprocessing(args)  # train_agent(args)



if __name__ == '__main__':
    GPU_ID = -1  # >=0 means GPU ID, -1 means CPU
    DRL_ID = 1
    alg_continuous_action_off_policy(GPU_ID, DRL_ID)
