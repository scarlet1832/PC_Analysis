# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Testing/RobinCalibration.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap


class Ui_RobinCalibration_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1097, 806)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.LidarSide = QtWidgets.QComboBox(self.centralwidget)
        self.LidarSide.setGeometry(QtCore.QRect(280, 90, 111, 25))
        self.LidarSide.setObjectName("LidarSide")
        self.LidarSide.addItem("")
        self.LidarSide.addItem("")
        self.LidarSide.addItem("")
        self.ChargeTimeCali = QtWidgets.QCheckBox(self.centralwidget)
        self.ChargeTimeCali.setGeometry(QtCore.QRect(150, 140, 91, 23))
        self.ChargeTimeCali.setObjectName("ChargeTimeCali")
        self.GeometricCali = QtWidgets.QCheckBox(self.centralwidget)
        self.GeometricCali.setGeometry(QtCore.QRect(240, 140, 91, 23))
        self.GeometricCali.setObjectName("GeometricCali")
        self.CrossTalkCali = QtWidgets.QCheckBox(self.centralwidget)
        self.CrossTalkCali.setGeometry(QtCore.QRect(330, 140, 121, 23))
        self.CrossTalkCali.setObjectName("CrossTalkCali")
        self.ReflectanceCali = QtWidgets.QCheckBox(self.centralwidget)
        self.ReflectanceCali.setGeometry(QtCore.QRect(460, 140, 101, 23))
        self.ReflectanceCali.setObjectName("ReflectanceCali")
        self.MinDistofChannelCali = QtWidgets.QCheckBox(self.centralwidget)
        self.MinDistofChannelCali.setGeometry(QtCore.QRect(570, 140, 151, 23))
        self.MinDistofChannelCali.setObjectName("MinDistofChannelCali")
        self.Initializelabel = QtWidgets.QLabel(self.centralwidget)
        self.Initializelabel.setGeometry(QtCore.QRect(150, 10, 161, 51))
        self.Initializelabel.setObjectName("Initializelabel")
        self.Logolabel = QtWidgets.QLabel(self.centralwidget)
        self.Logolabel.setGeometry(QtCore.QRect(10, 10, 121, 111))
        self.Logolabel.setObjectName("Logolabel")
        self.Init_StartCali = QtWidgets.QCommandLinkButton(self.centralwidget)
        self.Init_StartCali.setGeometry(QtCore.QRect(300, 20, 31, 31))
        self.Init_StartCali.setText("")
        self.Init_StartCali.setObjectName("Init_StartCali")
        self.GenerateTempYaml = QtWidgets.QPushButton(self.centralwidget)
        self.GenerateTempYaml.setGeometry(QtCore.QRect(150, 90, 111, 25))
        self.GenerateTempYaml.setObjectName("GenerateTempYaml")
        self.CheckFW = QtWidgets.QPushButton(self.centralwidget)
        self.CheckFW.setGeometry(QtCore.QRect(580, 90, 41, 25))
        self.CheckFW.setObjectName("CheckFW")
        self.CheckSDK = QtWidgets.QPushButton(self.centralwidget)
        self.CheckSDK.setGeometry(QtCore.QRect(410, 90, 41, 25))
        self.CheckSDK.setObjectName("CheckSDK")
        self.FWUpgrade = QtWidgets.QPushButton(self.centralwidget)
        self.FWUpgrade.setGeometry(QtCore.QRect(410, 40, 101, 25))
        self.FWUpgrade.setObjectName("FWUpgrade")
        self.CaliLogBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.CaliLogBrowser.setGeometry(QtCore.QRect(140, 430, 621, 311))
        self.CaliLogBrowser.setObjectName("CaliLogBrowser")
        self.textEdit = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit.setGeometry(QtCore.QRect(460, 90, 111, 31))
        self.textEdit.setObjectName("textEdit")
        self.textEdit_2 = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit_2.setGeometry(QtCore.QRect(630, 90, 111, 31))
        self.textEdit_2.setObjectName("textEdit_2")
        self.VisualizePC = QtWidgets.QPushButton(self.centralwidget)
        self.VisualizePC.setGeometry(QtCore.QRect(150, 220, 111, 25))
        self.VisualizePC.setObjectName("VisualizePC")
        self.CalibrationProgress = QtWidgets.QPushButton(self.centralwidget)
        self.CalibrationProgress.setGeometry(QtCore.QRect(150, 260, 111, 25))
        self.CalibrationProgress.setObjectName("CalibrationProgress")
        self.CloseVisualize = QtWidgets.QPushButton(self.centralwidget)
        self.CloseVisualize.setGeometry(QtCore.QRect(150, 300, 111, 25))
        self.CloseVisualize.setObjectName("CloseVisualize")
        self.ReVisualize = QtWidgets.QPushButton(self.centralwidget)
        self.ReVisualize.setGeometry(QtCore.QRect(150, 340, 111, 25))
        self.ReVisualize.setObjectName("ReVisualize")
        self.UploadTempYaml = QtWidgets.QPushButton(self.centralwidget)
        self.UploadTempYaml.setGeometry(QtCore.QRect(150, 180, 111, 25))
        self.UploadTempYaml.setObjectName("UploadTempYaml")
        self.CalibrationLogLabel = QtWidgets.QLabel(self.centralwidget)
        self.CalibrationLogLabel.setGeometry(QtCore.QRect(150, 390, 71, 31))
        self.CalibrationLogLabel.setObjectName("CalibrationLogLabel")
        self.CaliConfigBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.CaliConfigBrowser.setGeometry(QtCore.QRect(310, 200, 441, 171))
        self.CaliConfigBrowser.setObjectName("CaliConfigBrowser")
        self.CalibrationLogLabel_2 = QtWidgets.QLabel(self.centralwidget)
        self.CalibrationLogLabel_2.setGeometry(QtCore.QRect(310, 170, 101, 21))
        self.CalibrationLogLabel_2.setObjectName("CalibrationLogLabel_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.LidarSide.setItemText(0, _translate("MainWindow", "Lidar Side:0"))
        self.LidarSide.setItemText(1, _translate("MainWindow", "Lidar Side:1"))
        self.LidarSide.setItemText(2, _translate("MainWindow", "Lidar Side:2"))
        self.ChargeTimeCali.setText(_translate("MainWindow", "充电时间"))
        self.GeometricCali.setText(_translate("MainWindow", "几何标定"))
        self.CrossTalkCali.setText(_translate("MainWindow", "CrossTalk标定"))
        self.ReflectanceCali.setText(_translate("MainWindow", "反射率标定"))
        self.MinDistofChannelCali.setText(_translate("MainWindow", "通道最小距离标定"))
        self.Initializelabel.setText(_translate("MainWindow", "Robin标定程序初始化"))
        self.Logolabel.setText(_translate("MainWindow", "LogoLabel"))
        self.Logolabel.setScaledContents(True)
        self.Logolabel.setPixmap(QPixmap("/home/demo/Documents/DataAnalysis/GUI/SeyondLogo.png"))
        self.GenerateTempYaml.setText(_translate("MainWindow", "生成标定yaml"))
        self.CheckFW.setText(_translate("MainWindow", "FW"))
        self.CheckSDK.setText(_translate("MainWindow", "SDK"))
        self.FWUpgrade.setText(_translate("MainWindow", "FW upgrade"))
        self.VisualizePC.setText(_translate("MainWindow", "可视化点云"))
        self.CalibrationProgress.setText(_translate("MainWindow", "标定过程"))
        self.CloseVisualize.setText(_translate("MainWindow", "关闭可视化"))
        self.ReVisualize.setText(_translate("MainWindow", "重启可视化"))
        self.UploadTempYaml.setText(_translate("MainWindow", "上传标定yaml"))
        self.CalibrationLogLabel.setText(_translate("MainWindow", "标定日志"))
        self.CalibrationLogLabel_2.setText(_translate("MainWindow", "雷达标定参数"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_RobinCalibration_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())