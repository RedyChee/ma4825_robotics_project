#!/usr/bin/env python

import openravepy

if __name__ == "__main__":
    env = openravepy.Environment()
    env.Load('/home/redy/ma4825_ws/src/ahman_robot_openrave/worlds/pick_n_place.env.xml')
    env.SetViewer('qtcoin')
