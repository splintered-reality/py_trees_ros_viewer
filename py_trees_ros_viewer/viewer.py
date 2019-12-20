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

import datetime
import functools
import math
import os
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
    number_of_trees = len(demo_trees)
    send_tree.index = 0 if send_tree.index == (number_of_trees - 1) else send_tree.index + 1
    demo_trees[send_tree.index]['timestamp'] = time.time()
    console.logdebug("send: tree '{}' [{}][viewer]".format(
        send_tree.index, demo_trees[send_tree.index]['timestamp'])
    )
    javascript_command = "render_tree({{tree: {}}})".format(demo_trees[send_tree.index])
    web_view_page.runJavaScript(javascript_command, send_tree_response)


send_tree.index = 0


@qt_core.pyqtSlot()
def capture_screenshot(parent, web_engine_view, unused_checked):
    console.logdebug("captured screenshot [viewer]")
    file_dialog = qt_widgets.QFileDialog(parent)
    file_dialog.setNameFilters(
        [
            "BMP Files (*.bmp)",
            "JPEG Files (*.jpeg)",
            "PNG Files (*.png)"
        ]
    )
    file_dialog.selectNameFilter("PNG Files (*.png)")
    file_dialog.setDefaultSuffix((".png"))
    file_dialog.setAcceptMode(qt_widgets.QFileDialog.AcceptSave)
    # unfortunately creates a fair amount of spam on stdout
    #   'kf5.kio.core: Invalid URL: QUrl("screenshot.jpeg")'
    #   'kf5.kio.core: Invalid URL: QUrl("screenshot.png")'
    # but...it ain't broke
    file_dialog.selectFile("screenshot_{}.png".format(datetime.datetime.now().strftime("%S%M%H%d%m%y")))
    unused_result = file_dialog.exec()
    # should be able to restrict it to one file?
    for filename in file_dialog.selectedFiles():
        console.logdebug("capturing screenshot: {}".format(filename))
    # This would be simpler, but you can't specify a default filename, nor suffix on linux...
#     filename, _ = qt_widgets.QFileDialog.getSaveFileName(
#         parent=parent,
#         caption="Export to Png",
#         directory="screenshot_{}.png".format(datetime.datetime.now().strftime("%S%M%H%d%m%y")),
#         filter="BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png)",  # for multiple options, use ;;, e.g. 'All Files (*);;BMP Files (*.bmp);;JPEG Files (*.jpeg);;PNG Files (*.png)'
#         initialFilter="PNG Files (*.png)",
#         options=options
#     )
#     if filename:
#         console.loginfo("capturing screenshot: {}".format(filename))
        extension = os.path.splitext(filename)[-1].upper()
        if filename.endswith(".png"):
            extension = b'PNG'
        elif filename.endswith(".bmp"):
            extension = b'BMP'
        elif filename.endswith(".jpeg"):
            extension = b'JPEG'
        else:
            extension = b'PNG'
        web_engine_view.grab().save(filename, extension)


def on_blackboard_data_checked(backend, state: qt_core.Qt.Checked):
    with backend.lock:
        backend.parameters.blackboard_data = True if state == qt_core.Qt.Checked else False
    if state == qt_core.Qt.Checked:
        console.logdebug("Blackboard data requested")
    else:
        console.logdebug("Blackboard data disabled")


def on_blackboard_activity_checked(backend, state: qt_core.Qt.Checked):
    with backend.lock:
        backend.parameters.blackboard_activity = True if state == qt_core.Qt.Checked else False
    if state == qt_core.Qt.Checked:
        console.logdebug("Blackboard activity requested")
    else:
        console.logdebug("Blackboard activity disabled")


def on_periodic_checked(backend, state: qt_core.Qt.Checked):
    with backend.lock:
        backend.parameters.snapshot_period = 2.0 if state == qt_core.Qt.Checked else math.inf
    if state == qt_core.Qt.Checked:
        console.logdebug("Periodic snapshots requested")
    else:
        console.logdebug("Periodic snapshots disabled")


def on_connection_request(backend, namespace: str):
    """
    Enqueue a connection request.

    Cannot directly make the connection here since this is invariably the qt thread.
    """
    with backend.lock:
        backend.enqueued_connection_request_namespace = namespace

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
    snapshot_period = 2.0 if window.ui.periodic_checkbox.isChecked() else math.inf
    backend = ros_backend.Backend(
        parameters=ros_backend.SnapshotStream.Parameters(
            blackboard_data=window.ui.blackboard_data_checkbox.isChecked(),
            blackboard_activity=window.ui.blackboard_activity_checkbox.isChecked(),
            snapshot_period=snapshot_period)
    )

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
    window.ui.screenshot_button.clicked.connect(
        functools.partial(
            capture_screenshot,
            window,
            window.ui.web_view_group_box.ui.web_engine_view,
        )
    )
    window.ui.blackboard_data_checkbox.stateChanged.connect(
        functools.partial(
            on_blackboard_data_checked,
            backend,
        )
    )
    window.ui.blackboard_activity_checkbox.stateChanged.connect(
        functools.partial(
            on_blackboard_activity_checked,
            backend,
        )
    )
    window.ui.periodic_checkbox.stateChanged.connect(
        functools.partial(
            on_periodic_checked,
            backend,
        )
    )
    window.ui.topic_combo_box.currentTextChanged.connect(
        functools.partial(
            on_connection_request,
            backend
        )
    )

    backend.discovered_namespaces_changed.connect(window.on_discovered_namespaces_changed)
    backend.tree_snapshot_arrived.connect(window.on_tree_snapshot_arrived)
    # two signals for the combo box are relevant
    #   activated - only when there is a user interaction
    #   currentTextChanged - when there is a programmatic OR user interaction
    # window.ui.topic_combo_box.activated.connect(backend.connect)
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
