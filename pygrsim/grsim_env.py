# coding=utf-8
# Copyright 2019 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""GRSim Environment."""


from pygrsim import pygrsim


def create_environment(env_name='',
                       rewards='scoring',
                       render=False,
                       write_video=False,
                       dump_frequency=1,
                       logdir='',
                       number_of_left_players_agent_controls=1,
                       number_of_right_players_agent_controls=0,
                       dt=0.016):
    """Creates a GRSim environment.
    Args:
      env_name: a name of a scenario to run, e.g. "11_vs_11_stochastic".
        The list of scenarios can be found in directory "scenarios".
      rewards: Comma separated list of rewards to be added.
         Currently supported rewards are 'scoring' and 'checkpoints'.
      render: whether to render game frames.
         Must be enable when rendering videos or when using pixels
         representation.
      write_video: whether to dump videos when a trace is dumped.
      dump_frequency: how often to write dumps/videos (in terms of # of episodes)
        Sub-sample the episodes for which we dump videos to save some disk space.
      logdir: directory holding the logs.
      number_of_left_players_agent_controls: Number of left players an agent
          controls.
      number_of_right_players_agent_controls: Number of right players an agent
          controls.
      dt: the time steps of simulator
    Returns:
      GRSim environment.
    """
    assert env_name

    env = pygrsim.SSLWorld(env_name, rewards, write_video, logdir,
                           number_of_left_players_agent_controls,
                           number_of_right_players_agent_controls,
                           dt)
    if render:
        env.render()
    return env