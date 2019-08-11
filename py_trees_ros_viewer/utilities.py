#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_viewer/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Demo trees to feed to the web app
"""
##############################################################################
# Imports
##############################################################################

import time
import typing

import rclpy.node
import rclpy.qos

##############################################################################
# Methods
##############################################################################


def find_topics(
        node: rclpy.node.Node,
        topic_type: str,
        namespace: str=None,
        timeout: float=0.5) -> typing.List[str]:
    """
    Discover a topic of the specified type and if necessary, under the specified
    namespace.

    .. note: This method has been reproduced from py_trees_ros.utilities to decouple
    dependencies between the viewer and the core py_trees libraries.

    Args:
        node: nodes have the discovery methods
        topic_type: primary lookup hint
        namespace: secondary lookup hint
        timeout: check every 0.1s until this timeout is reached (can be None -> checks once)

    .. note: Immediately post node creation, it can take some time to discover the graph.

    Returns:
        list of fully expanded topic names (can be empty)
    """
    # TODO: follow the pattern of ros2cli to create a node without the need to init
    # rcl (might get rid of the magic sleep this way). See:
    #    https://github.com/ros2/ros2cli/blob/master/ros2service/ros2service/verb/list.py
    #    https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/node/strategy.py
    loop_period = 0.1  # seconds
    clock = rclpy.clock.Clock()
    start_time = clock.now()
    topic_names = []
    while True:
        # Returns a list of the form: [('exchange/blackboard', ['std_msgs/String'])
        topic_names_and_types = node.get_topic_names_and_types()
        topic_names = [name for name, types in topic_names_and_types if topic_type in types]
        if namespace is not None:
            topic_names = [name for name in topic_names if namespace in name]
        if topic_names:
            break
        if timeout is None or (clock.now() - start_time) > rclpy.time.Duration(seconds=timeout):
            break
        else:
            time.sleep(loop_period)
    return topic_names


def qos_profile_latched_topic() -> rclpy.qos.QoSProfile:
    """
    Convenience retrieval for a latched topic (publisher / subscriber)
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    )


def normalise_name_strings(name) -> str:
    """
    To prepare them for a json dump, they need to have newlines and
    superflous apostrophe's removed.

    Args:
        name: name to normalise
    """
    return name.replace('\n', ' ')
