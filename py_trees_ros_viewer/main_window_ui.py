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
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.dock_widget_contents)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.testing_group_box = QtWidgets.QGroupBox(self.dock_widget_contents)
        self.testing_group_box.setObjectName("testing_group_box")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.testing_group_box)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.send_button = QtWidgets.QPushButton(self.testing_group_box)
        self.send_button.setEnabled(False)
        self.send_button.setObjectName("send_button")
        self.verticalLayout_2.addWidget(self.send_button)
        self.verticalLayout_3.addWidget(self.testing_group_box)
        self.topic_selector_group_box = QtWidgets.QGroupBox(self.dock_widget_contents)
        self.topic_selector_group_box.setObjectName("topic_selector_group_box")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.topic_selector_group_box)
        self.verticalLayout.setObjectName("verticalLayout")
        self.topic_combo_box = QtWidgets.QComboBox(self.topic_selector_group_box)
        self.topic_combo_box.setInsertPolicy(QtWidgets.QComboBox.InsertAlphabetically)
        self.topic_combo_box.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.topic_combo_box.setObjectName("topic_combo_box")
        self.verticalLayout.addWidget(self.topic_combo_box)
        self.verticalLayout_3.addWidget(self.topic_selector_group_box)
        spacerItem = QtWidgets.QSpacerItem(20, 218, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.dock_widget.setWidget(self.dock_widget_contents)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(1), self.dock_widget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PyTrees Viewer"))
        self.web_view_group_box.setTitle(_translate("MainWindow", "Tree View"))
        self.testing_group_box.setTitle(_translate("MainWindow", "Testing"))
        self.send_button.setText(_translate("MainWindow", "Send Tree"))
        self.topic_selector_group_box.setTitle(_translate("MainWindow", "Topic Selector"))

from py_trees_ros_viewer.web_view import WebViewGroupBox
from . import images_rc
