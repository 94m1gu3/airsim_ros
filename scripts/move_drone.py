#!/usr/bin/env python

from os import path

import rospy
import rospkg
from std_msgs.msg import Bool

import airsim
from ast import literal_eval

from utils import reset_objects_id, set_objects_id


def prepare_start(client):
    reset_objects_id(client)
    set_objects_id(client)

    # Async methods returns Future. Call join() to wait for task to complete.
    client.takeoffAsync().join()


def move_for(client, points):
    for position in points:
        x, y, z, velocity = position
        print("Moving to {0} at velocity {1}".format((x, y, z), velocity))
        
        # move to position
        client.moveToPositionAsync(x, y, z, velocity).join()


def read_points(file='points.txt'):
    points = []
    with open(file, 'r') as fd:
        for line in fd.readlines():
            if line.startswith('#'): continue # this is a comment
            point = literal_eval(line)
            points.append(point)
    return points


if __name__ == "__main__":

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('airsim_ros')
    
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    rospy.init_node('moving_drone', anonymous=True)
    publisher = rospy.Publisher('/stop_message', Bool, queue_size=1)

    prepare_start(client)
    print(pkg_path)
    points = read_points(path.join(pkg_path, 'scripts/points.txt'))
    move_for(client, points)

    # give time for the last images
    rate = rospy.Rate(10) # 30hz
    rate.sleep()

    publisher.publish(Bool(data=True))

    client.armDisarm(False)
    client.reset()

    # that's enough fun for now. let's quit cleanly
    client.enableApiControl(False)
