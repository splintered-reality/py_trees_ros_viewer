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
Launch a qt dashboard for the tutorials.
"""
##############################################################################
# Imports
##############################################################################

import functools
import json
import time

import PyQt5.QtCore as qt_core
import PyQt5.QtWidgets as qt_widgets

from . import main_window_ui
from . import console

##############################################################################
# Helpers
##############################################################################


class MainWindow(qt_widgets.QMainWindow):

    request_shutdown = qt_core.pyqtSignal(name="requestShutdown")
    topic_selected_automagically = qt_core.pyqtSignal(str, name="topicSelectedAutomagically")

    def __init__(self, default_tree):
        super().__init__()
        self.ui = main_window_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.readSettings()
        self.ui.web_view_group_box.ui.web_engine_view.loadFinished.connect(
            functools.partial(
                self.onLoadFinished,
                default_tree
            )
        )

    @qt_core.pyqtSlot(list)
    def onDiscoveredTopicsChanged(self, discovered_topics):
        console.logdebug("discovered topics changed callback {}[main window]".format(discovered_topics))

        discovered_topics.sort()

        current_selection = self.ui.topic_combo_box.currentText()
        if current_selection:
            console.logdebug("currently selected topic: {}".format(current_selection))
        else:
            console.logdebug("currently selected topic: none")

        # if current_selection and current_selection not in discovered_topics:
        #     discovered_topics.append(current_selection)

        for topic in discovered_topics:
            if self.ui.topic_combo_box.findText(topic) < 0:
                self.ui.topic_combo_box.addItem(topic)
        for index in range(self.ui.topic_combo_box.count()):
            topic = self.ui.topic_combo_box.itemText(index)
            if topic != current_selection:
                if topic not in discovered_topics:
                    self.ui.topic_combo_box.removeItem(index)

        if current_selection:
            self.ui.topic_combo_box.setCurrentText(current_selection)
        else:
            self.ui.topic_combo_box.setCurrentIndex(0)
            # can't seem to get at the str version of this, always requires the int
            # self.ui.topic_combo_box.activated.emit(discovered_topics[0])
            # self.topic_selected_automagically.emit(discovered_topics[0])

    @qt_core.pyqtSlot()
    def onLoadFinished(self, default_tree):
        console.logdebug("web page loaded [main window]")
        self.ui.send_button.setEnabled(True)

        def handle_response(reply):
            console.logdebug("reply: '{}' [viewer]".format(reply))

        default_tree['timestamp'] = time.time()
        self.ui.web_view_group_box.ui.web_engine_view.page().runJavaScript(
            "render_tree({tree: '%s'});" % json.dumps(default_tree),
            handle_response
        )

    def closeEvent(self, event):
        console.logdebug("received close event [main_window]")
        self.request_shutdown.emit()
        self.writeSettings()
        super().closeEvent(event)

    def readSettings(self):
        console.logdebug("read settings [main_window]")
        settings = qt_core.QSettings("Splintered Reality", "PyTrees Viewer")
        geometry = settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        window_state = settings.value("window_state")  # full size, maximised, minimised, no state
        if window_state is not None:
            self.restoreState(window_state)

    def writeSettings(self):
        console.logdebug("write settings [main_window]")
        settings = qt_core.QSettings("Splintered Reality", "PyTrees Viewer")
        settings.setValue("geometry", self.saveGeometry())
        settings.setValue("window_state", self.saveState())  # full size, maximised, minimised, no state
