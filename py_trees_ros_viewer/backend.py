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

import os

import PyQt5.QtCore as qt_core

import py_trees_ros
import rclpy

from . import console

##############################################################################
# Helpers
##############################################################################

##############################################################################
# Backend
##############################################################################


class Backend(qt_core.QObject):

    # led_colour_changed = qt_core.pyqtSignal(str, name="ledColourChanged")
    # safety_sensors_enabled_changed = qt_core.pyqtSignal(bool, name="safetySensorsEnabledChanged")
    # battery_percentage_changed = qt_core.pyqtSignal(float, name="batteryPercentageChanged")
    # battery_charging_status_changed = qt_core.pyqtSignal(float, name="batteryChargingStatusChanged")

    def __init__(self):
        super().__init__()
        default_node_name = "tree_viewer_" + str(os.getpid())
        self.node = rclpy.create_node(default_node_name)
        self.shutdown_requested = False
        self.topic_type_string = 'py_trees_ros_interfaces/msg/BehaviourTree'

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            self.discover_publishers()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [backend]")
        self.shutdown_requested = True

    def discover_publishers(self):
        topic_names = py_trees_ros.utilities.find_topics(
            node=self.node,
            topic_type=self.topic_type_string,
            namespace=None,
            timeout=0.1  # seconds
        )
        console.logdebug("topic names: {} [backend]".format(topic_names))
