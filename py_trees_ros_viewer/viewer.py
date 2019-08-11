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
A qt-javascript application for viewing executing or replaying py_trees
"""
##############################################################################
# Imports
##############################################################################

import functools
import json
import signal
import sys
import threading
import time

import PyQt5.QtCore as qt_core
import PyQt5.QtWidgets as qt_widgets

import py_trees_js
import rclpy

from . import backend as ros_backend
from . import console
from . import main_window

##############################################################################
# Helpers
##############################################################################


def send_tree_response(reply):
    console.logdebug("reply: '{}' [viewer]".format(reply))


@qt_core.pyqtSlot()
def send_tree(web_view_page, demo_trees, unused_checked):
    send_tree.index = 0 if send_tree.index == 2 else send_tree.index + 1
    demo_trees[send_tree.index]['timestamp'] = time.time()
    console.logdebug("send: tree '{}' [{}][viewer]".format(
        send_tree.index, demo_trees[send_tree.index]['timestamp'])
    )
    javascript_command = "render_tree({{tree: {}}})".format(demo_trees[send_tree.index])
    web_view_page.runJavaScript(javascript_command, send_tree_response)


send_tree.index = 0

##############################################################################
# Main
##############################################################################


def main():
    # logging
    console.log_level = console.LogLevel.DEBUG

    # ros init
    rclpy.init()

    # the players
    app = qt_widgets.QApplication(sys.argv)
    demo_trees = py_trees_js.viewer.trees.create_demo_tree_list()
    window = main_window.MainWindow()
    backend = ros_backend.Backend()

    # sig interrupt handling
    #   use a timer to get out of the gui thread and
    #   permit python a chance to catch the signal
    #   https://stackoverflow.com/questions/4938723/what-is-the-correct-way-to-make-my-pyqt-application-quit-when-killed-from-the-co
    def on_shutdown(unused_signal, unused_frame):
        console.logdebug("received interrupt signal [viewer]")
        window.close()

    signal.signal(signal.SIGINT, on_shutdown)
    timer = qt_core.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(250)

    # sigslots
    window.ui.send_button.clicked.connect(
        functools.partial(
             send_tree,
             window.ui.web_view_group_box.ui.web_engine_view.page(),
             demo_trees
        )
    )
    backend.discovered_topics_changed.connect(window.on_discovered_topics_changed)
    backend.tree_snapshot_arrived.connect(window.on_tree_snapshot_arrived)
    # two signals for the combo box are relevant
    #   activated - only when there is a user interaction
    #   currentTextChanged - when there is a programmatic OR user interaction
    # window.ui.topic_combo_box.activated.connect(backend.connect)
    window.ui.topic_combo_box.currentTextChanged.connect(backend.connect)
    window.request_shutdown.connect(backend.terminate_ros_spinner)

    # qt/ros bringup
    ros_thread = threading.Thread(target=backend.spin)
    ros_thread.start()
    window.show()
    result = app.exec_()

    # shutdown
    backend.node.get_logger().info("waiting for backend to terminate [viewer]")
    ros_thread.join()
    rclpy.shutdown()
    sys.exit(result)
