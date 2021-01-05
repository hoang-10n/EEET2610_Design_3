# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'UIi.ui'
#
# Created by: PyQt5 UI code generator 5.15.1
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import QtTest
from PyQt5.QtWidgets import (QApplication, QWidget)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.Qt import Qt
import file_rc
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import base64
import cv2
import numpy as np

broker_location = "192.168.4.1"

global pixmap
pixmap = None

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        self.x = 240
        self.y = 210
        self.i = 0
        self.angle = 10
        self.control = False
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(903, 595)
        MainWindow.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        MainWindow.setAutoFillBackground(False)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(80, 250, 71, 71))
        font = QtGui.QFont()
        font.setPointSize(35)
        self.pushButton.setFont(font)
        self.pushButton.setAutoFillBackground(False)
        self.pushButton.setStyleSheet("background-color: rgb(170, 170, 0);")
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(80, 320, 71, 71))
        font = QtGui.QFont()
        font.setPointSize(31)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setAutoFillBackground(False)
        self.pushButton_2.setStyleSheet("background-color: rgb(170, 170, 0);")
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(150, 290, 71, 61))
        font = QtGui.QFont()
        font.setPointSize(35)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setAutoFillBackground(False)
        self.pushButton_3.setStyleSheet("background-color: rgb(170, 170, 0);")
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(10, 290, 71, 61))
        font = QtGui.QFont()
        font.setPointSize(35)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setAutoFillBackground(False)
        self.pushButton_4.setStyleSheet("background-color: rgb(170, 170, 0);")
        self.pushButton_4.setObjectName("pushButton_4")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(50, 410, 131, 41))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAutoFillBackground(False)
        self.label.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label.setScaledContents(False)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(10, 10, 221, 221))
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap(":/newPrefix/tải xuống.jpg"))
        self.label_2.setScaledContents(True)
        self.label_2.setObjectName("label_2")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(40, 470, 151, 61))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(True)
        font.setUnderline(True)
        font.setWeight(75)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setStyleSheet("background-color: rgb(170, 0, 0);\n"
                                        "background-color: rgb(66, 199, 0);")
        self.pushButton_5.setObjectName("pushButton_5")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(240, 210, 16, 16))
        self.label_4.setMaximumSize(QtCore.QSize(151, 16777215))
        self.label_4.setText("")
        self.label_4.setPixmap(QtGui.QPixmap(":/newPrefix/12.png"))
        self.label_4.setScaledContents(True)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(70, 110, 101, 21))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(250, 250, 631, 321))
        self.label_6.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.label_6.setFrameShadow(QtWidgets.QFrame.Plain)
        self.label_6.setLineWidth(11)
        self.label_6.setMidLineWidth(0)
        self.label_6.setText("")
        self.label_6.setPixmap(QtGui.QPixmap(":/newPrefix/vv.png"))
        self.label_6.setScaledContents(True)
        self.label_6.setWordWrap(False)
        self.label_6.setIndent(-1)
        self.label_6.setObjectName("label_6")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(660, 20, 211, 201))
        self.label_7.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.label_7.setText("")
        self.label_7.setPixmap(QtGui.QPixmap(":/newPrefix/car.png"))
        self.label_7.setScaledContents(True)
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(800, 110, 31, 16))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(690, 110, 31, 16))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(750, 200, 31, 16))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        self.label_10.setFont(font)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setGeometry(QtCore.QRect(750, 20, 31, 16))
        font = QtGui.QFont()
        font.setPointSize(11)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        font.setStrikeOut(False)
        self.label_11.setFont(font)
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(650, 11, 231, 221))
        self.graphicsView.setStyleSheet("background-color: rgb(67, 200, 200);")
        self.graphicsView.setFrameShape(QtWidgets.QFrame.Box)
        self.graphicsView.setFrameShadow(QtWidgets.QFrame.Plain)
        self.graphicsView.setObjectName("graphicsView")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(240, 10, 401, 221))
        self.label_3.setText("")
        self.label_3.setPixmap(QtGui.QPixmap(":/newPrefix/map.PNG"))
        self.label_3.setScaledContents(True)
        self.label_3.setAlignment(QtCore.Qt.AlignLeading | QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.label_3.setWordWrap(False)
        self.label_3.setObjectName("label_3")
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setGeometry(QtCore.QRect(380, 50, 141, 131))
        self.label_12.setText("")
        self.label_12.setPixmap(QtGui.QPixmap(":/newPrefix/compasss.png"))
        self.label_12.setScaledContents(True)
        self.label_12.setObjectName("label_12")
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setGeometry(QtCore.QRect(420, 90, 61, 61))
        self.label_13.setText("")
        self.label_13.setScaledContents(True)
        self.label_13.setObjectName("label_13")

        self.t = QtGui.QTransform()
        self.t.rotate(self.angle)

        self.label_13.setPixmap(QtGui.QPixmap(":/newPrefix/arr.png").transformed(self.t))

        self.pushButton.setCheckable(True)
        self.pushButton.clicked.connect(self.btnstate)
        self.pushButton_2.setCheckable(True)
        self.pushButton_2.clicked.connect(self.btnstate)
        self.pushButton_3.setCheckable(True)
        self.pushButton_3.clicked.connect(self.btnstate)
        self.pushButton_4.setCheckable(True)
        self.pushButton_4.clicked.connect(self.btnstate)
        self.pushButton_5.setCheckable(True)
        self.pushButton_5.clicked.connect(self.btnstate)
        self.label_3.raise_()
        self.label_12.raise_()
        self.graphicsView.raise_()
        self.pushButton.raise_()
        self.pushButton_2.raise_()
        self.pushButton_3.raise_()
        self.pushButton_4.raise_()
        self.label.raise_()
        self.label_2.raise_()
        self.pushButton_5.raise_()
        self.label_5.raise_()
        self.label_6.raise_()
        self.label_7.raise_()
        self.label_8.raise_()
        self.label_9.raise_()
        self.label_10.raise_()
        self.label_11.raise_()
        self.label_4.raise_()
        self.label_13.raise_()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.carMove)
        self.timer.start(100)

        self.timer2 = QtCore.QTimer()
        self.timer2.timeout.connect(self.changeImage)
        self.timer2.start(60)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def btnstate(self):
        if self.pushButton.isChecked():
            if self.pushButton_5.isChecked():
                self.label.setText("Status: Up")
                publish.single("FirstTry/Test", "Up", hostname = broker_location)
                self.y -= 10
            self.pushButton.toggle()
        elif self.pushButton_2.isChecked():
            if self.pushButton_5.isChecked():
                self.y += 10
                self.label.setText("Status: Down")
                publish.single("FirstTry/Test", "Down", hostname = broker_location)
            self.pushButton_2.toggle()
        elif self.pushButton_3.isChecked():
            if self.pushButton_5.isChecked():
                self.x += 10
                self.label.setText("Status: Right")
                publish.single("FirstTry/Test", "Right", hostname = broker_location)
            self.pushButton_3.toggle()
        elif self.pushButton_4.isChecked():
            if self.pushButton_5.isChecked():
                self.x -= 10
                self.label.setText("Status: Left")
                publish.single("FirstTry/Test", "Left", hostname = broker_location)
            self.pushButton_4.toggle()
        elif self.pushButton_5.isChecked():
            self.pushButton_5.setText("Automatic Control")
            self.control = True
        elif not self.pushButton_5.isChecked():
            self.pushButton_5.setText("Manual Control")
            self.label.setText("Status: N_A")
            self.control = False
        self.label_4.move(self.x, self.y)

    def carMove(self):
        if not self.control:
            if self.i < 15:
                self.x += 26
            elif self.i < 25:
                self.y -= 20
            elif self.i < 40:
                self.x -= 26
            elif self.i < 50:
                self.y += 20
            if self.i == 49:
                self.i = -1
            self.label_4.move(self.x, self.y)
            self.t.rotate(self.angle)
            self.label_13.setPixmap(QtGui.QPixmap(":/newPrefix/arr.png").transformed(self.t))
            self.i = self.i + 1

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_A:
            self.pushButton_4.toggle()
            self.btnstate()
        elif event.key() == Qt.Key_D:
            self.pushButton_3.toggle()
            self.btnstate()
        elif event.key() == Qt.Key_W:
            self.pushButton.toggle()
            self.btnstate()
        elif event.key() == Qt.Key_S:
            self.pushButton_2.toggle()
            self.btnstate()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "↑"))
        self.pushButton_2.setText(_translate("MainWindow", "↓"))
        self.pushButton_3.setText(_translate("MainWindow", "→"))
        self.pushButton_4.setText(_translate("MainWindow", "←"))
        self.label.setText(_translate("MainWindow", "Staus:"))
        self.pushButton_5.setText(_translate("MainWindow", "Maunnal Control"))
        self.label_5.setText(_translate("MainWindow", "0"))
        self.label_8.setText(_translate("MainWindow", "0"))
        self.label_9.setText(_translate("MainWindow", "0"))
        self.label_10.setText(_translate("MainWindow", "0"))
        self.label_11.setText(_translate("MainWindow", "0"))

    def changeImage(self):
        global pixmap
        if (pixmap is not None):
            img = base64.b64decode(pixmap)
            npimg = np.frombuffer(img, dtype=np.uint8)
            source = cv2.imdecode(npimg, 1)
            image = QImage(source.data, source.shape[1], source.shape[0], source.shape[1]*3, QImage.Format_BGR888)
            self.label_6.setPixmap(QPixmap.fromImage(image))


class Viewer:
    def on_message(self, client, userdata, message):
        global pixmap
        pixmap = message.payload
        # self.stream(message.payload)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected")
        self.client.subscribe("Design3/Camera")
    
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker_location)
        self.client.loop_start()


if __name__ == "__main__":
    import sys
    viewer = Viewer()
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
