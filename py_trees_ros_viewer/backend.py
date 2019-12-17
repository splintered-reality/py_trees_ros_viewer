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
Ros backend for the viewer.
"""
##############################################################################
# Imports
##############################################################################

import copy
import os
import time

import PyQt5.QtCore as qt_core

import py_trees_ros_interfaces.msg as py_trees_msgs
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy

from . import console
from . import conversions
from . import utilities

##############################################################################
# Helpers
##############################################################################


class Parameters(object):
    """
    Local representation of dynamic parameter for streaming
    snapshot data. This is critical to have around to store the
    user's desired settings so that when a connection is made,
    they can be automagically applied.
    """

    def __init__(self):
        self.snapshot_blackboard_data = False
        self.snapshot_blackboard_activity = False

##############################################################################
# Backend
##############################################################################


class Backend(qt_core.QObject):

    discovered_topics_changed = qt_core.pyqtSignal(list, name="discoveredTopicsChanged")
    tree_snapshot_arrived = qt_core.pyqtSignal(dict, name="treeSnapshotArrived")

    def __init__(self, parameters):
        super().__init__()
        default_node_name = "tree_viewer_" + str(os.getpid())
        self.node = rclpy.create_node(default_node_name)
        self.shutdown_requested = False
        self.topic_type = py_trees_msgs.BehaviourTree
        self.topic_type_string = 'py_trees_ros_interfaces/msg/BehaviourTree'
        self.discovered_topics = []
        self.discovered_timestamp = time.monotonic()
        self.discovery_loop_time_sec = 3.0
        self.subscription = None
        self.cached_blackboard = {"behaviours": {}, "data": {}}
        self.parameter_client = None
        self.parameters = parameters

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            self.discover_topics()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [backend]")
        self.shutdown_requested = True

    def discover_topics(self):
        timeout = self.discovered_timestamp + self.discovery_loop_time_sec
        if self.discovered_topics and (time.monotonic() < timeout):
            return
        new_topic_names = utilities.find_topics(
            node=self.node,
            topic_type=self.topic_type_string,
            namespace=None,
            timeout=None  # oneshot check
        )
        new_topic_names.sort()
        if self.discovered_topics != new_topic_names:
            self.discovered_topics = new_topic_names
            self.discovered_topics_changed.emit(self.discovered_topics)
            console.logdebug("discovered topics changed {}[backend]".format(self.discovered_topics))
        self.discovered_timestamp = time.monotonic()

    def connect(self, topic_name: str):
        """
        Cancel any current subscriptions and create a new subscription to the specified topic.

        Args:
            topic_name: fully qualified name of topic to subscribe to
        """
        if self.subscription:
            console.logdebug("cancelling existing subscription [{}][backend]".format(self.subscription))
            self.node.destroy_subscription(self.subscription)
            self.node.destroy_client(self.parameter_client)
        console.logdebug("creating a new subscription [{}][backend]".format(topic_name))
        self.subscription = self.node.create_subscription(
            msg_type=self.topic_type,
            topic=topic_name,
            callback=self.tree_snapshot_handler,
            qos_profile=utilities.qos_profile_latched_topic()
        )
        # dynamic parameter client
        self.parameter_client = self.node.create_client(
            rcl_srvs.SetParameters,
            '/tree/set_parameters'  # TODO: dynamically get the namespace from topic_name and reconstruct
        )
        self.dynamically_reconfigure_parameters(
            snapshot_blackboard_data=self.parameters.snapshot_blackboard_data,
            snapshot_blackboard_activity=self.parameters.snapshot_blackboard_activity
        )

    def dynamically_reconfigure_parameters(
            self,
            snapshot_blackboard_data=None,
            snapshot_blackboard_activity=None
    ):
        if self.parameter_client is not None:
            request = rcl_srvs.SetParameters.Request()  # noqa
            if snapshot_blackboard_data is not None:
                parameter = rcl_msgs.Parameter()
                parameter.name = "snapshot_blackboard_data"
                parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
                parameter.value.bool_value = snapshot_blackboard_data
                request.parameters.append(parameter)
            if snapshot_blackboard_activity is not None:
                parameter = rcl_msgs.Parameter()
                parameter.name = "snapshot_blackboard_activity"
                parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
                parameter.value.bool_value = snapshot_blackboard_activity
                request.parameters.append(parameter)
            # unused_future = self.parameter_client.call_async(request)

    def snapshot_blackboard_data(self, snapshot: bool):
        if self.parameter_client is not None:
            request = rcl_srvs.SetParameters.Request()  # noqa
            parameter = rcl_msgs.Parameter()
            parameter.name = "snapshot_blackboard_data"
            parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
            parameter.value.bool_value = snapshot
            request.parameters.append(parameter)
            unused_future = self.parameter_client.call_async(request)
        self.parameters.snapshot_blackboard_data = snapshot

    def tree_snapshot_handler(self, msg: py_trees_msgs.BehaviourTree):
        """
        Callback to receive incoming tree snapshots before relaying them to the web application.

        Args:
            msg: incoming serialised tree snapshot

        Note: this uses a clever(?) hack to accumulate visited path snapshots of the blackboard
        to gain a representation of the entire blackboard without having to transmit the
        entire blackboard on every update. Special care is needed to make sure what has been
        removed from the blackboard (does not get transmitted), actually gets removed.
        """
        console.logdebug("handling incoming tree snapshot [backend]")
        colours = {
            'Sequence': '#FFA500',
            'Selector': '#00FFFF',
            'Parallel': '#FFFF00',
            'Behaviour': '#555555',
            'Decorator': '#DDDDDD',
        }
        tree = {
            'changed': "true" if msg.changed else "false",
            'timestamp': msg.statistics.stamp.sec + float(msg.statistics.stamp.nanosec) / 1.0e9,
            'behaviours': {},
            'blackboard': {'behaviours': {}, 'data': {}},
            'visited_path': []}
        # hack, update the blackboard from visited path contexts
        blackboard_variables = {}
        for blackboard_variable in msg.blackboard_on_visited_path:
            blackboard_variables[blackboard_variable.key] = blackboard_variable.value
        for behaviour in msg.behaviours:
            behaviour_id = str(conversions.msg_to_uuid4(behaviour.own_id))
            behaviour_type = conversions.msg_constant_to_behaviour_str(behaviour.type)
            if behaviour.is_active:
                tree['visited_path'].append(behaviour_id)
            tree['behaviours'][behaviour_id] = {
                'id': behaviour_id,
                'status': conversions.msg_constant_to_status_str(behaviour.status),
                'name': utilities.normalise_name_strings(behaviour.name),
                'colour': colours[behaviour_type],
                'children': [str(conversions.msg_to_uuid4(child_id)) for child_id in behaviour.child_ids],
                'data': {
                    'Class': behaviour.class_name,
                    'Feedback': behaviour.message,
                },
            }
            if behaviour.blackboard_access:
                variables = []
                for variable in behaviour.blackboard_access:
                    variables.append(variable.key + " ({})".format(variable.value))
                tree['behaviours'][behaviour_id]['data']['Blackboard'] = variables
                tree['blackboard']['behaviours'][behaviour_id] = {variable.key: variable.value}
                # hack, update the blackboard from visited path contexts
                if (
                    variable.key in self.cached_blackboard and
                    variable.value != 'r' and
                    variable.key not in blackboard_variables
                ):
                    del self.cached_blackboard[variable.key]
        # hack, update the blackboard from visited path contexts
        self.cached_blackboard.update(blackboard_variables)
        tree['blackboard']['data'] = copy.deepcopy(self.cached_blackboard)
        self.tree_snapshot_arrived.emit(tree)
