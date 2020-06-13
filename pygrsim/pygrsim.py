from ctypes import *
import numpy as np
from numpy.ctypeslib import as_ctypes
import configparser
import os
import gym

base_path = os.path.dirname(__file__)
sslsim_lib = cdll.LoadLibrary(os.path.join(base_path, '../lib/libsslsim.so'))

MAX_ROBOT_COUNT = 16
TEAM_COUNT = 2


class Action(Structure):
    pass


Action._fields_ = [
    ('team_id', c_int),
    ('robot_id', c_int),
    ('vel', c_double * 3),
    ('spin', c_int),
    ('kick', c_double * 2),
]


class Observation(Structure):
    pass


Observation._fields_ = [
    ('ball', c_double * 3),
    ('robots', (((c_double * 3) * MAX_ROBOT_COUNT) * TEAM_COUNT)),
    ('kick', (((c_int * 2) * MAX_ROBOT_COUNT) * TEAM_COUNT)),
]

sslsim_lib.SSLWorld_new.argtypes = None
sslsim_lib.SSLWorld_new.restype = c_void_p

sslsim_lib.SSL_del.argtypes = [c_void_p]
sslsim_lib.SSL_del.restype = None

sslsim_lib.step.argtypes = [c_void_p, POINTER(Action), POINTER(Action), c_int, c_int]
sslsim_lib.step.restype = Observation

sslsim_lib.reset.argtypes = [c_void_p]
sslsim_lib.reset.restype = Observation


class SSLWorld(gym.Env):
    def __init__(self, env_name, rewards, write_video, logdir):
        self.config = configparser.ConfigParser()
        self.config.read(os.path.join(base_path, '../bin/grsim_simulator.ini'))
        print(self.config.sections())
        self.obj = sslsim_lib.SSLWorld_new()

    def __del__(self):
        sslsim_lib.SSL_del(self.obj)

    def step(self, action_dict):
        actions_0 = [None for _ in range(int(self.config['game_vars']['blue_robots_count']))]
        actions_1 = [None for _ in range(int(self.config['game_vars']['yellow_robots_count']))]

        for k, v in enumerate(action_dict):
            team = str(k)[0]
            robot_id = int(str(k).split('_')[1])
            if team == 'y':
                actions_0[robot_id] = v
            elif team == 'b':
                actions_1[robot_id] = v
            else:
                print('Wrong Action keywords')

        act_0 = Action()
        # act_0.vel = [v for v in action_0.]
        # return sslsim_lib.step(self.obj, act_0, act_1)

    def reset(self):
        return sslsim_lib.reset(self.obj)