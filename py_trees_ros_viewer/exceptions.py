#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Custom exception types for py_trees_ros_viewer.
"""

##############################################################################
# Imports
##############################################################################


class NotReadyError(Exception):
    """
    Typically used when methods have been called that expect, but have not
    pre-engaged in the ROS2 specific setup typical of py_trees_ros classes
    and behaviours.
    """
    pass


class TimedOutError(Exception):
    """
    Timed out waiting (typically) for middleware connections to be established.
    """
    pass
