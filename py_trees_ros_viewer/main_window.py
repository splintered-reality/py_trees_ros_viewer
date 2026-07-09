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

    def __init__(self):
        super().__init__()
        self.ui = main_window_ui.Ui_MainWindow()
        self.ui.setupUi(self)
        self.readSettings()
        self.ui.web_view_group_box.ui.web_engine_view.loadFinished.connect(self.onLoadFinished)
        self.web_app_loaded = False
        self.pre_loaded_tree = None
        self.pending_tree = None
        self.render_in_flight = False

    @qt_core.pyqtSlot(dict)
    def on_tree_snapshot_arrived(self, tree):
        """
        Coalesce incoming snapshots - only ever have one render in flight and
        keep no more than the latest arrival pending. Without this, snapshots
        arriving faster than the web app can render them (e.g., large blackboard
        payloads) queue without bound and the app appears to hang.
        """
        if self.web_app_loaded:
            self.pending_tree = tree
            if not self.render_in_flight:
                self.render_pending_tree()
        else:
            self.pre_loaded_tree = tree

    def render_pending_tree(self):
        tree = self.pending_tree
        self.pending_tree = None
        if tree is None:
            return
        self.render_in_flight = True
        javascript_command = "render_tree({{tree: {}}})".format(tree)
        web_view_page = self.ui.web_view_group_box.ui.web_engine_view.page()
        web_view_page.runJavaScript(javascript_command, self.on_tree_rendered)

    def on_tree_rendered(self, response):
        """
        Callback triggered on a response being received from the js render_tree method.

        Kicks off a render of the most recently arrived snapshot if one queued
        up while this render was in flight.
        """
        console.logdebug("response from js/render_tree ['{}'][window]".format(response))
        self.render_in_flight = False
        self.render_pending_tree()

    @qt_core.pyqtSlot()
    def on_connection_reset(self):
        """
        Clear the timeline cache so that snapshots from a previous connection
        (e.g., pre-restart of the tree application) don't pollute the timeline.
        """
        self.pending_tree = None
        if self.web_app_loaded:
            web_view_page = self.ui.web_view_group_box.ui.web_engine_view.page()
            web_view_page.runJavaScript("reset_timeline()", self.on_timeline_reset)
        else:
            self.pre_loaded_tree = None

    def on_timeline_reset(self, response):
        """
        Callback triggered on a response being received from the js reset_timeline method.
        """
        console.logdebug("response from js/reset_timeline ['{}'][window]".format(response))

    @qt_core.pyqtSlot(list)
    def on_discovered_namespaces_changed(self, discovered_namespaces):
        console.logdebug("discovered namespaces changed callback {}[window]".format(discovered_namespaces))
        if (discovered_namespaces):
            self.ui.send_button.setEnabled(False)
        else:
            self.ui.send_button.setEnabled(True)

        discovered_namespaces.sort()

        current_selection = self.ui.topic_combo_box.currentText()
        if current_selection:
            console.logdebug("currently selected namespace: {}".format(current_selection))
        else:
            console.logdebug("currently selected namespace: none")

        # if current_selection and current_selection not in discovered_topics:
        #     discovered_topics.append(current_selection)

        for namespace in discovered_namespaces:
            if self.ui.topic_combo_box.findText(namespace) < 0:
                self.ui.topic_combo_box.addItem(namespace)
        for index in range(self.ui.topic_combo_box.count()):
            topic = self.ui.topic_combo_box.itemText(index)
            if topic != current_selection:
                if topic not in discovered_namespaces:
                    self.ui.topic_combo_box.removeItem(index)

        if current_selection:
            self.ui.topic_combo_box.setCurrentText(current_selection)
        else:
            self.ui.topic_combo_box.setCurrentIndex(0)
            # can't seem to get at the str version of this, always requires the int
            # self.ui.topic_combo_box.activated.emit(discovered_namespaces[0])
            # self.topic_selected_automagically.emit(discovered_namespaces[0])

    @qt_core.pyqtSlot()
    def onLoadFinished(self):
        console.logdebug("web page loaded [window]")
        self.web_app_loaded = True
        self.ui.send_button.setEnabled((self.ui.topic_combo_box.currentIndex() == -1))
        self.ui.screenshot_button.setEnabled(True)
        self.ui.blackboard_activity_checkbox.setEnabled(True)
        self.ui.blackboard_data_checkbox.setEnabled(True)
        self.ui.periodic_checkbox.setEnabled(True)
        if self.pre_loaded_tree:
            self.pending_tree = self.pre_loaded_tree
            self.pre_loaded_tree = None
            if not self.render_in_flight:
                self.render_pending_tree()

    def closeEvent(self, event):
        console.logdebug("received close event [window]")
        self.request_shutdown.emit()
        self.writeSettings()
        super().closeEvent(event)

    def readSettings(self):
        console.logdebug("read settings [window]")
        settings = qt_core.QSettings("Splintered Reality", "PyTrees Viewer")
        geometry = settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        window_state = settings.value("window_state")  # full size, maximised, minimised, no state
        if window_state is not None:
            self.restoreState(window_state)
        self.ui.blackboard_data_checkbox.setChecked(
            settings.value("blackboard_data", defaultValue=True, type=bool)
        )
        self.ui.blackboard_activity_checkbox.setChecked(
            settings.value("blackboard_activity", defaultValue=False, type=bool)
        )
        self.ui.periodic_checkbox.setChecked(
            settings.value("periodic", defaultValue=True, type=bool)
        )

    def writeSettings(self):
        console.logdebug("write settings [window]")
        settings = qt_core.QSettings("Splintered Reality", "PyTrees Viewer")
        settings.setValue("geometry", self.saveGeometry())
        settings.setValue("window_state", self.saveState())  # full size, maximised, minimised, no state
        settings.setValue(
            "blackboard_data",
            self.ui.blackboard_data_checkbox.isChecked()
        )
        settings.setValue(
            "blackboard_activity",
            self.ui.blackboard_activity_checkbox.isChecked()
        )
        settings.setValue(
            "periodic",
            self.ui.periodic_checkbox.isChecked()
        )
