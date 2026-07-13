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
import math
import os
import threading
import time
import typing

import PyQt5.QtCore as qt_core

import py_trees_ros_interfaces.msg as py_trees_msgs
import py_trees_ros_interfaces.srv as py_trees_srvs
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import rclpy.node

from . import console
from . import conversions
from . import exceptions
from . import utilities

##############################################################################
# Helpers
##############################################################################


class SnapshotStream(object):
    """
    The tree watcher sits on the other side of a running
    :class:`~py_trees_ros.trees.BehaviourTree` and manages the dynamic
    connection of a snapshot stream.
    """

    class Parameters(object):
        """
        Reconfigurable parameters for the snapshot stream.

        Args:
            blackboard_data: publish blackboard variables on the visited path
            blackboard_activity: enable and publish blackboard activity in the last tick
            snapshot_period: period between snapshots (use /inf to only publish on tree status changes)
        """
        def __init__(
            self,
            blackboard_data: bool=False,
            blackboard_activity: bool=False,
            snapshot_period: float=math.inf
        ):
            self.blackboard_data = blackboard_data
            self.blackboard_activity = blackboard_activity
            self.snapshot_period = snapshot_period

        def __eq__(self, other):
            return ((self.blackboard_data == other.blackboard_data) and
                    (self.blackboard_activity == other.blackboard_activity) and
                    (self.snapshot_period == other.snapshot_period)
                    )

    def __init__(
        self,
        node: rclpy.node.Node,
        namespace: str,
        parameters: 'SnapshotStream.Parameters',
        callback: typing.Callable[[py_trees_msgs.BehaviourTree], None],
    ):
        """
        Args:
            namespace: connect to the snapshot stream services in this namespace
            parameters: snapshot stream configuration controlling both on-the-fly stream creation and display
            statistics: display statistics

        .. seealso:: :mod:`py_trees_ros.programs.tree_watcher`
        """

        self.namespace = namespace
        self.parameters = copy.copy(parameters) if parameters is not None else SnapshotStream.Parameters()
        self.node = node
        self.callback = callback

        self.topic_name = None
        self.subscriber = None

        self.services = {
            'open': None,
            'close': None,
            'reconfigure': None
        }

        self.service_names = {
            'open': self.namespace + "/open",
            'close': self.namespace + "/close",
            'reconfigure': self.namespace + "/reconfigure",
        }
        self.service_type_strings = {
            'open': 'py_trees_ros_interfaces/srv/OpenSnapshotStream',
            'close': 'py_trees_ros_interfaces/srv/CloseSnapshotStream',
            'reconfigure': 'py_trees_ros_interfaces/srv/ReconfigureSnapshotStream'
        }
        self.service_types = {
            'open': py_trees_srvs.OpenSnapshotStream,
            'close': py_trees_srvs.CloseSnapshotStream,
            'reconfigure': py_trees_srvs.ReconfigureSnapshotStream
        }
        # create service clients and the connection, cleaning up behind
        # itself if any step fails (e.g., the tree application vanished)
        try:
            self.services["open"] = self.create_service_client(key="open")
            self.services["close"] = self.create_service_client(key="close")
            self.services["reconfigure"] = self.create_service_client(key="reconfigure")
            self._connect_on_init()
        except exceptions.TimedOutError:
            self.destroy_communications()
            raise

    def reconfigure(self, parameters: 'SnapshotStream.Parameters'):
        """
        Reconfigure the stream.

        Args:
            parameters: new configuration
        """
        if self.parameters == parameters:
            return
        self.parameters = copy.copy(parameters)
        request = self.service_types["reconfigure"].Request()
        request.topic_name = self.topic_name
        request.parameters.blackboard_data = self.parameters.blackboard_data
        request.parameters.blackboard_activity = self.parameters.blackboard_activity
        request.parameters.snapshot_period = self.parameters.snapshot_period
        unused_future = self.services["reconfigure"].call_async(request)

    def _connect_on_init(self, timeout_sec=1.0):
        """
        Request a snapshot stream and make a connection to it.

        Args:
            timeout_sec: how long to hold on making connections

        Raises:
            :class:`~py_trees_ros.exceptions.NotReadyError`: if setup() wasn't called to identify the relevant services to connect to.
            :class:`~py_trees_ros.exceptions.TimedOutError`: if it times out waiting for the server
        """
        # request a stream
        request = self.service_types["open"].Request()
        request.parameters.blackboard_data = self.parameters.blackboard_data
        request.parameters.blackboard_activity = self.parameters.blackboard_activity
        request.parameters.snapshot_period = self.parameters.snapshot_period
        console.logdebug("establishing a snapshot stream connection [{}][backend]".format(self.namespace))
        future = self.services["open"].call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise exceptions.TimedOutError(
                "timed out waiting for a response from '{}'".format(self.service_names["open"])
            )
        self.topic_name = response.topic_name
        # connect to a snapshot stream
        start_time = time.monotonic()
        while True:
            elapsed_time = time.monotonic() - start_time
            if elapsed_time > timeout_sec:
                raise exceptions.TimedOutError("timed out waiting for a snapshot stream publisher [{}]".format(self.topic_name))
            if self.node.count_publishers(self.topic_name) > 0:
                break
            time.sleep(0.1)
        self.subscriber = self.node.create_subscription(
            msg_type=py_trees_msgs.BehaviourTree,
            topic=self.topic_name,
            callback=self.callback,
            qos_profile=utilities.qos_profile_latched()
        )
        console.logdebug("  ...ok [backend]")

    def shutdown(self):
        if (
            rclpy.ok() and
            self.topic_name is not None and
            self.services["close"] is not None and
            self.services["close"].service_is_ready()
        ):
            request = self.service_types["close"].Request()
            request.topic_name = self.topic_name
            future = self.services["close"].call_async(request)
            rclpy.spin_until_future_complete(
                node=self.node,
                future=future,
                timeout_sec=0.5)
            unused_response = future.result()
        self.destroy_communications()

    def destroy_communications(self):
        """
        Destroy the subscriber and service clients so they don't linger
        on the node after this stream has been discarded.
        """
        if self.subscriber is not None:
            self.node.destroy_subscription(self.subscriber)
            self.subscriber = None
        for key, client in self.services.items():
            if client is not None:
                self.node.destroy_client(client)
                self.services[key] = None

    def create_service_client(self, key: str):
        """
        Convenience api for opening a service client and waiting for the service to appear.

        Args:
            key: one of 'open', 'close'.

        Raises:
            :class:`~py_trees_ros.exceptions.NotReadyError`: if setup() wasn't called to identify the relevant services to connect to.
            :class:`~py_trees_ros.exceptions.TimedOutError`: if it times out waiting for the server
        """
        if self.service_names[key] is None:
            raise exceptions.NotReadyError(
                "no known '{}' service known [did you call setup()?]".format(self.service_types[key])
            )
        client = self.node.create_client(
            srv_type=self.service_types[key],
            srv_name=self.service_names[key],
            qos_profile=rclpy.qos.qos_profile_services_default
        )
        # hardcoding timeouts will get us into trouble
        if not client.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError(
                "timed out waiting for {}".format(self.service_names['close'])
            )
        return client

##############################################################################
# Backend
##############################################################################


class Backend(qt_core.QObject):

    discovered_namespaces_changed = qt_core.pyqtSignal(list, name="discoveredNamespacesChanged")
    tree_snapshot_arrived = qt_core.pyqtSignal(dict, name="treeSnapshotArrived")
    connection_reset = qt_core.pyqtSignal(name="connectionReset")

    def __init__(self, parameters):
        super().__init__()
        default_node_name = "tree_viewer_" + str(os.getpid())
        self.node = rclpy.create_node(default_node_name)
        self.shutdown_requested = False
        self.snapshot_stream_type = py_trees_msgs.BehaviourTree
        self.discovered_namespaces = []
        self.discovered_timestamp = time.monotonic()
        self.discovery_loop_time_sec = 3.0
        self.cached_blackboard = {"behaviours": {}, "data": {}}
        self.snapshot_stream = None
        self.connected_namespace = None
        self.parameters = parameters

        self.lock = threading.Lock()
        self.enqueued_connection_request_namespace = None

    def spin(self):
        with self.lock:
            old_parameters = copy.copy(self.parameters)
        while rclpy.ok() and not self.shutdown_requested:
            self.discover_namespaces()
            # hold the lock only long enough to snapshot state shared with the
            # qt thread - connecting can block for seconds at a time and the
            # qt handlers block on this lock (a frozen gui ensues otherwise)
            with self.lock:
                if self.parameters != old_parameters:
                    if self.snapshot_stream is not None:
                        self.snapshot_stream.reconfigure(self.parameters)
                old_parameters = copy.copy(self.parameters)
                enqueued_namespace = self.enqueued_connection_request_namespace
                self.enqueued_connection_request_namespace = None
            if enqueued_namespace is not None:
                self.connect(enqueued_namespace)
            else:
                self.maintain_connection()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.snapshot_stream is not None:
            self.snapshot_stream.shutdown()
        self.node.destroy_node()

    def terminate_ros_spinner(self):
        self.node.get_logger().info("shutdown requested [backend]")
        self.shutdown_requested = True

    def discover_namespaces(self):
        """
        Oneshot lookup for namespaces within which snapshot stream services exist.
        This is additionally conditioned on 'discovery_loop_time_sec' so that it
        doesn't spam the check at the same rate as the node is spinning.

        If a change in the result occurs, it emits a signal for the qt ui.
        """
        timeout = self.discovered_timestamp + self.discovery_loop_time_sec
        if self.discovered_namespaces and (time.monotonic() < timeout):
            return
        open_service_type_string = "py_trees_ros_interfaces/srv/OpenSnapshotStream"
        service_names_and_types = self.node.get_service_names_and_types()
        new_service_names = [name for name, types in service_names_and_types if open_service_type_string in types]
        new_service_names.sort()
        new_namespaces = [utilities.parent_namespace(name) for name in new_service_names]
        if self.discovered_namespaces != new_namespaces:
            self.discovered_namespaces = new_namespaces
            self.discovered_namespaces_changed.emit(self.discovered_namespaces)
            console.logdebug("discovered namespaces changed {}[backend]".format(self.discovered_namespaces))
        self.discovered_timestamp = time.monotonic()

    def connect(self, namespace):
        """
        Cancel the current connection and create a new one to the specified namespace.

        If the connection attempt fails (e.g., the tree application disappeared
        in the meantime), it will be retried via :meth:`maintain_connection`
        as soon as the snapshot stream services are rediscovered.

        Args:
            namespace: in which to find snapshot stream services
        """
        if self.snapshot_stream is not None:
            console.logdebug("cancelling existing snapshot stream connection [{}][backend]".format(self.snapshot_stream))
            self.snapshot_stream.shutdown()
            self.snapshot_stream = None
        self.connected_namespace = namespace
        self.cached_blackboard = {"behaviours": {}, "data": {}}
        console.logdebug("creating a new snapshot stream connection [{}][backend]".format(namespace))
        with self.lock:
            parameters = copy.copy(self.parameters)
        try:
            self.snapshot_stream = SnapshotStream(
                node=self.node,
                namespace=namespace,
                callback=self.tree_snapshot_handler,
                parameters=parameters
            )
            self.connection_reset.emit()
        except exceptions.TimedOutError as e:
            console.logwarn("failed to connect, will retry when services reappear [{}][{}][backend]".format(namespace, str(e)))

    def maintain_connection(self):
        """
        Check the health of the current snapshot stream and reconnect if it died.

        When the tree application restarts, it does so with new snapshot
        stream services and topics (albeit under the same namespace), leaving
        this viewer connected to topics that no longer have a publisher. Detect
        that and re-establish the connection as soon as the services reappear.
        """
        if self.connected_namespace is None:
            return
        if self.snapshot_stream is not None:
            if self.snapshot_stream.topic_name is None:
                return
            if self.node.count_publishers(self.snapshot_stream.topic_name) == 0:
                console.logwarn("lost connection to the snapshot stream [{}][backend]".format(self.snapshot_stream.topic_name))
                self.snapshot_stream.shutdown()
                self.snapshot_stream = None
        if self.snapshot_stream is None:
            # wait for the full set of services - connecting while the tree is
            # only partially through its setup otherwise triggers an endless
            # cycle of blocking, timing-out connection attempts
            required_service_names = [
                self.connected_namespace + "/" + suffix
                for suffix in ("open", "close", "reconfigure")
            ]
            service_names = [name for name, unused_types in self.node.get_service_names_and_types()]
            if all(name in service_names for name in required_service_names):
                console.loginfo("snapshot stream services rediscovered, reconnecting [{}][backend]".format(self.connected_namespace))
                self.connect(self.connected_namespace)

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
            'Composite': '#9070DD',
            'Behaviour': '#555555',
            'Decorator': '#DDDDDD',
        }
        tree = {
            'changed': "true" if msg.changed else "false",
            'timestamp': msg.statistics.stamp.sec + float(msg.statistics.stamp.nanosec) / 1.0e9,
            'behaviours': {},
            'blackboard': {'behaviours': {}, 'data': {}},
            'visited_path': []
        }
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
                'details': behaviour.additional_detail,
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
                    tree['blackboard']['behaviours'].setdefault(behaviour_id, {})[variable.key] = variable.value
                tree['behaviours'][behaviour_id]['data']['Blackboard'] = variables
                # delete keys from the cache if they aren't in the visited variables list when
                # they should be (i.e. their parent behaviour is on the visited path and has
                # 'w' or 'x' permissions on the variable).
                if (
                    variable.key in self.cached_blackboard and
                    variable.value != 'r' and
                    behaviour.is_active and
                    variable.key not in blackboard_variables
                ):
                    del self.cached_blackboard[variable.key]
        # hack, update the blackboard from visited path contexts
        self.cached_blackboard.update(blackboard_variables)
        if self.snapshot_stream.parameters.blackboard_data:
            tree['blackboard']['data'] = copy.deepcopy(self.cached_blackboard)
        if self.snapshot_stream.parameters.blackboard_activity:
            xhtml = utilities.XhtmlSymbols()
            xhtml_snippet = "<table>"
            for item in msg.blackboard_activity:
                if item.activity_type == "READ":
                    info = xhtml.normal + xhtml.left_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "WRITE":
                    info = xhtml.green + xhtml.right_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "ACCESSED":
                    info = xhtml.yellow + xhtml.left_right_arrow + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "ACCESS_DENIED":
                    info = xhtml.red + xhtml.multiplication_x + xhtml.space + "client has no read/write access" + xhtml.reset
                elif item.activity_type == "NO_KEY":
                    info = xhtml.red + xhtml.multiplication_x + xhtml.space + "key does not yet exist" + xhtml.reset
                elif item.activity_type == "NO_OVERWRITE":
                    info = xhtml.yellow + xhtml.forbidden_circle + xhtml.space + item.current_value + xhtml.reset
                elif item.activity_type == "UNSET":
                    info = ""
                elif item.activity_type == "INITIALISED":
                    info = xhtml.green + xhtml.right_arrow + xhtml.space + item.current_value + xhtml.reset
                else:
                    info = ""
                xhtml_snippet += (
                    "<tr>"
                    "<td>" + xhtml.cyan + item.key + xhtml.reset + "</td>"
                    "<td style='text-align: center;'>" + xhtml.yellow + item.activity_type + xhtml.reset + "</td>"
                    "<td style='text-align: center;'>" + xhtml.normal + item.client_name + xhtml.reset + "</td>"
                    "<td>" + info + "</td>"
                    "</tr>"
                )
            xhtml_snippet += "</table>"
            tree['activity'] = [xhtml_snippet]

        self.tree_snapshot_arrived.emit(tree)
