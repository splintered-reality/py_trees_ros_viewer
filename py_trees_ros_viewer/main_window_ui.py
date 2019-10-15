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
        MainWindow.resize(799, 356)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/tuxrobot.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        self.central_display = QtWidgets.QWidget(MainWindow)
        self.central_display.setObjectName("central_display")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.central_display)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.tools_frame = QtWidgets.QFrame(self.central_display)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tools_frame.sizePolicy().hasHeightForWidth())
        self.tools_frame.setSizePolicy(sizePolicy)
        self.tools_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.tools_frame.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.tools_frame.setObjectName("tools_frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.tools_frame)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.tools_frame)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.topic_combo_box = QtWidgets.QComboBox(self.tools_frame)
        self.topic_combo_box.setInsertPolicy(QtWidgets.QComboBox.InsertAlphabetically)
        self.topic_combo_box.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.topic_combo_box.setObjectName("topic_combo_box")
        self.horizontalLayout.addWidget(self.topic_combo_box)
        spacerItem = QtWidgets.QSpacerItem(186, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.screenshot_button = QtWidgets.QPushButton(self.tools_frame)
        self.screenshot_button.setEnabled(False)
        self.screenshot_button.setObjectName("screenshot_button")
        self.horizontalLayout.addWidget(self.screenshot_button)
        self.send_button = QtWidgets.QPushButton(self.tools_frame)
        self.send_button.setEnabled(False)
        self.send_button.setObjectName("send_button")
        self.horizontalLayout.addWidget(self.send_button)
        self.verticalLayout_4.addWidget(self.tools_frame)
        self.web_view_group_box = WebViewGroupBox(self.central_display)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.web_view_group_box.sizePolicy().hasHeightForWidth())
        self.web_view_group_box.setSizePolicy(sizePolicy)
        self.web_view_group_box.setTitle("")
        self.web_view_group_box.setObjectName("web_view_group_box")
        self.verticalLayout_4.addWidget(self.web_view_group_box)
        MainWindow.setCentralWidget(self.central_display)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 799, 35))
        self.menubar.setDefaultUp(False)
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PyTrees Viewer"))
        self.label.setText(_translate("MainWindow", "Topic"))
        self.screenshot_button.setText(_translate("MainWindow", "Screenshot"))
        self.send_button.setText(_translate("MainWindow", "Send Demo Tree"))

from py_trees_ros_viewer.web_view import WebViewGroupBox
from . import images_rc
