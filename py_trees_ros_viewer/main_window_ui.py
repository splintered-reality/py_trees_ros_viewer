# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_window.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(551, 356)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/tuxrobot.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.central_display = QtWidgets.QWidget(MainWindow)
        self.central_display.setObjectName("central_display")
        self.central_horizontal_layout = QtWidgets.QHBoxLayout(self.central_display)
        self.central_horizontal_layout.setObjectName("central_horizontal_layout")
        self.web_view_group_box = WebViewGroupBox(self.central_display)
        self.web_view_group_box.setObjectName("web_view_group_box")
        self.central_horizontal_layout.addWidget(self.web_view_group_box)
        MainWindow.setCentralWidget(self.central_display)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 551, 29))
        self.menubar.setDefaultUp(False)
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.dock_widget = QtWidgets.QDockWidget(MainWindow)
        self.dock_widget.setObjectName("dock_widget")
        self.dock_widget_contents = QtWidgets.QWidget()
        self.dock_widget_contents.setObjectName("dock_widget_contents")
        self.dock_vertical_layout = QtWidgets.QVBoxLayout(self.dock_widget_contents)
        self.dock_vertical_layout.setObjectName("dock_vertical_layout")
        self.send_button = QtWidgets.QPushButton(self.dock_widget_contents)
        self.send_button.setEnabled(False)
        self.send_button.setObjectName("send_button")
        self.dock_vertical_layout.addWidget(self.send_button)
        spacerItem = QtWidgets.QSpacerItem(20, 218, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.dock_vertical_layout.addItem(spacerItem)
        self.dock_widget.setWidget(self.dock_widget_contents)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(1), self.dock_widget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PyTrees Viewer"))
        self.web_view_group_box.setTitle(_translate("MainWindow", "Tree View"))
        self.send_button.setText(_translate("MainWindow", "Send Tree"))

from py_trees_ros_viewer.web_view import WebViewGroupBox
from . import images_rc
