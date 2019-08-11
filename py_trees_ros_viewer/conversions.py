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

import uuid

import py_trees_ros_interfaces.msg as py_trees_msgs
import unique_identifier_msgs.msg as unique_identifier_msgs

##############################################################################
# Methods
##############################################################################


def msg_to_uuid4(msg: unique_identifier_msgs.UUID) -> uuid.UUID:
    """
    Convert a uuid4 python object to a ros unique identifier, UUID type.

    Args:
        msg: the ros message type

    Returns:
        the behaviour's uuid, python style
    """
    return uuid.UUID(bytes=bytes(msg.uuid), version=4)


def msg_constant_to_behaviour_str(value: int) -> str:
    """
    Convert one of the behaviour type constants in a
    :class:`py_trees_ros_interfaces.msg.Behaviour` message to
    a human readable string.

    Args:
        value: see the message definition for details

    Returns:
        the bheaviour class type as a string (e.g. 'Sequence')

    Raises:
        TypeError: if the message type is unrecognised
    """
    if value == py_trees_msgs.Behaviour.SEQUENCE:
        return 'Sequence'
    elif value == py_trees_msgs.Behaviour.CHOOSER:
        return 'Chooser'
    elif value == py_trees_msgs.Behaviour.SELECTOR:
        return 'Selector'
    elif value == py_trees_msgs.Behaviour.PARALLEL:
        return 'Parallel'
    elif value == py_trees_msgs.Behaviour.DECORATOR:
        return 'Decorator'
    elif value == py_trees_msgs.Behaviour.BEHAVIOUR:
        return 'Behaviour'
    else:
        raise TypeError("invalid type specified in message [{}]".format(value))


def msg_constant_to_status_str(value: int) -> str:
    """
    Convert one of the status constants in a
    :class:`py_trees_ros_interfaces.msg.Behaviour` message to
    a human readable string.

    Args:
        value: see the message definition for details

    Returns:
        status as a string ('Invalid', 'Failure', 'Running', 'Success')

    Raises:
        TypeError: if the status type is unrecognised
    """
    if value == py_trees_msgs.Behaviour.INVALID:
        return 'INVALID'
    elif value == py_trees_msgs.Behaviour.RUNNING:
        return 'RUNNING'
    elif value == py_trees_msgs.Behaviour.SUCCESS:
        return 'SUCCESS'
    elif value == py_trees_msgs.Behaviour.FAILURE:
        return 'FAILURE'
    else:
        raise TypeError("invalid status specified in message [{}]".format(value))
