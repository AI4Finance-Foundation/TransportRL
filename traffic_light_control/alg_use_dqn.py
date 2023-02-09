import sys
import gym
import numpy as np
from elegantrl.train.run import train_agent
from elegantrl.train.config import Config
from elegantrl.agents.AgentDQN import AgentDQN, AgentDuelingDQN, AgentDoubleDQN

from config import INTERSECTIONS
from env import TrafficLightControlEnv
from config import ROAD_MODEL

def alg_discrete_action_off_policy(gpu_id):
    assert ROAD_MODEL in [0, 1]
    agent_class = [AgentDQN, AgentDuelingDQN, AgentDoubleDQN, AgentDoubleDQN][0]
    env_class = TrafficLightControlEnv
    env = TrafficLightControlEnv()
    env_args = {
        'env_num': 1,
        'env_name': 'TrafficLightControl',
        'max_step': 1000,
        'state_dim': env.calc_state_dim(),
        'action_dim': 2,
        'if_discrete': True,
        'target_return': +np.inf,
    }
    args = Config(agent_class, env_class=env_class, env_args=env_args)
    args.env = env
    args.target_step = args.max_step
    args.net_dim = (64, 32)
    args.batch_size = 256

    args.gamma = 0.97
    args.eval_times = 2 ** 3
    args.eval_gap = 2 ** 4


    args.learner_gpus = gpu_id
    args.random_seed += gpu_id

    train_agent(args)


if __name__ == '__main__':
    GPU_ID = int(sys.argv[1] if len(sys.argv) > 1 else 0)
    print("GPU_ID: ", GPU_ID)

    alg_discrete_action_off_policy(GPU_ID)
