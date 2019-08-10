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
import time

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

    discovered_topics_changed = qt_core.pyqtSignal(list, name="discoveredTopicsChanged")

    def __init__(self):
        super().__init__()
        default_node_name = "tree_viewer_" + str(os.getpid())
        self.node = rclpy.create_node(default_node_name)
        self.shutdown_requested = False
        self.topic_type_string = 'py_trees_ros_interfaces/msg/BehaviourTree'
        self.discovered_topics = []
        self.discovered_timestamp = time.monotonic()
        self.discovery_loop_time_sec = 3.0

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
        new_topic_names = py_trees_ros.utilities.find_topics(
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
        else:
            console.logdebug("nochange")
        self.discovered_timestamp = time.monotonic()
