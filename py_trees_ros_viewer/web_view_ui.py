# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'web_view.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_WebViewGroupBox(object):
    def setupUi(self, WebViewGroupBox):
        WebViewGroupBox.setObjectName("WebViewGroupBox")
        WebViewGroupBox.resize(400, 300)
        self.web_view_layout = QtWidgets.QVBoxLayout(WebViewGroupBox)
        self.web_view_layout.setObjectName("web_view_layout")
        self.web_engine_view = QtWebEngineWidgets.QWebEngineView(WebViewGroupBox)
        self.web_engine_view.setUrl(QtCore.QUrl("qrc:/index.html"))
        self.web_engine_view.setObjectName("web_engine_view")
        self.web_view_layout.addWidget(self.web_engine_view)

        self.retranslateUi(WebViewGroupBox)
        QtCore.QMetaObject.connectSlotsByName(WebViewGroupBox)

    def retranslateUi(self, WebViewGroupBox):
        _translate = QtCore.QCoreApplication.translate
        WebViewGroupBox.setWindowTitle(_translate("WebViewGroupBox", "Tree View"))
        WebViewGroupBox.setTitle(_translate("WebViewGroupBox", "GroupBox"))

from PyQt5 import QtWebEngineWidgets
from . import web_app_rc
