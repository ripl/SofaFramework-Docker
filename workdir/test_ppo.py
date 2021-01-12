#!/usr/bin/env python3
"""Example evaluation script to evaluate a policy.
This is an example evaluation script which loads a policy trained with PPO. If
this script were moved into the top rrc_simulation folder (since this is where
we will execute the rrc_evaluate command), it would consistute a valid
submission (naturally, imports below would have to be adjusted accordingly).
This script will be executed in an automated procedure.  For this to work, make
sure you do not change the overall structure of the script!
This script expects the following arguments in the given order:
 - Difficulty level (needed for reward computation)
 - initial pose of the cube (as JSON string)
 - goal pose of the cube (as JSON string)
 - file to which the action log is written
It is then expected to initialize the environment with the given initial pose
and execute exactly one episode with the policy that is to be evaluated.
When finished, the action log, which is created by the TriFingerPlatform class,
is written to the specified file.  This log file is crucial as it is used to
evaluate the actual performance of the policy.
"""
import json

import dl
import simple_control_policy
from dl.rl import set_env_to_eval_mode
from dl import nest
from dl.rl.algorithms.ppo2 import PPO2
import torch
import numpy as np





difficulty2config = {
    1: {'ppo': True, 'action_space': 'torque_and_position', 'adjust_tip': True},
    2: {'ppo': True, 'action_space': 'torque_and_position', 'adjust_tip': True},
    3: {'ppo': True, 'action_space': 'torque_and_position', 'adjust_tip': True},
    4: {'ppo': True, 'action_space': 'torque_and_position', 'adjust_tip': False}
}


def use_ppo(difficulty):
    # return difficulty in [1, 2, 3, 4] and False
    return difficulty2config[difficulty]['ppo']


def _init_env_and_policy(expdir, local_test=False):

    from utils import set_seed
    set_seed(0)
    dl.load_config(expdir + '/config.gin')
    ppo = PPO2(expdir, nenv=1)
    ppo.load()
    env = ppo.env
    set_env_to_eval_mode(env)
    env.record_episode = True
    # override env with fixed initializer
    # env.unwrapped.envs[0].unwrapped.initializer = initializer
    return env, ppo


def main(args):
    #goal_pose = move_cube.sample_goal(2)
    #goal_pose_json = json.dumps({
    #    'position': goal_pose.position.tolist(),
    #    'orientation': goal_pose.orientation.tolist()
    #})

    local_test = True  # set to True to turn on sim and visualization flags
    env, ppo = _init_env_and_policy(args.expdir,
                                    local_test=local_test)

    for _ in range(args.neps):
        done = False
        obs = env.reset()
        rnn_state = None
        while not done:
            obs = nest.map_structure(
                lambda x: torch.from_numpy(x).to(ppo.device), obs
            )
            with torch.no_grad():
                outs = ppo.pi(obs, state_in=rnn_state)
            action = nest.map_structure(lambda x: x.cpu().numpy(), outs.action)
            rnn_state = outs.state_out
            # if args.zero:
            #    if isinstance(env.envs[0].env, DiscreteMPFCResidualWrapper):
            #        action = nest.map_structure(
            #            lambda x: env.envs[0].n_ac // 2 * np.ones_like(x), action
            #        )
            #    else:
            action = nest.map_structure(lambda x: np.zeros_like(x), action)

            obs, reward, done, info = env.step(action)
            print(reward, action, obs)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('expdir', type=str)
    parser.add_argument('--neps', '-n', type=int, default=1)
    parser.add_argument('--zero', default=False, action='store_true')

    args = parser.parse_args()
    main(args)


