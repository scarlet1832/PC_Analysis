import pandas as pd
from Analysis.Recv_Publish_Cali_Data import Recv_Publish
from GUI.Rviz_Analysis_main import Ui_Rviz_Analysis
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5 import QtGui, QtWidgets
import sys
from io import StringIO
import subprocess
from PyQt5.QtGui import QIcon

# Set the display options.
pd.set_option('display.max_columns', None)
pd.set_option('display.expand_frame_repr', False)
pd.set_option('max_colwidth', None)


class TextBrowserRedirector(StringIO):
    def __init__(self, text_browser):
        super().__init__()
        self.text_browser = text_browser

    def write(self, text):
        self.text_browser.append(text)
class Main(QMainWindow, Ui_Rviz_Analysis):

    def __init__(self):
        super(Main, self).__init__()
        self.setupUi(self)
        self.setWindowIcon(QIcon('/home/demo/Documents/DataAnalysis/GUI/SeyondLogo.png'))
        self.text_redirector = TextBrowserRedirector(self.textBrowser)
        self.pushButton.clicked.connect(self.on_click)
        self.pushButton_2.clicked.connect(self.on_click2)
        self.pushButton_3.clicked.connect(self.on_click3)
        self.checkBox.stateChanged.connect(lambda: self.doCheck(0))
        self.checkBox_2.stateChanged.connect(lambda: self.doCheck1(1))
        self.checkBox_3.stateChanged.connect(lambda: self.doCheck2(2))
        self.checkBox_4.stateChanged.connect(lambda: self.doCheck3(3))
        self.checkBox_5.stateChanged.connect(lambda: self.doCheck4(4))
        self.checkBox_6.stateChanged.connect(lambda: self.doCheck5(5))
        self.checkBox_7.stateChanged.connect(lambda: self.doCheck6(6))
        self.checkBox_8.stateChanged.connect(lambda: self.doCheck7(7))
        self.checkBox_9.stateChanged.connect(lambda: self.doCheck8(8))
        self.checkBox_10.stateChanged.connect(lambda: self.doCheck10(10))
        self.checkBox_11.stateChanged.connect(lambda: self.doCheck11(11))
        self.checkBox_12.stateChanged.connect(lambda: self.doCheck12(12))
        self.checkBox_13.stateChanged.connect(lambda: self.doCheck13(13))
        self.comboBox.activated.connect(self.changeTopic)
        self.subscribe = Recv_Publish()
        # sys.stdout = TextBrowserRedirector(self.textBrowser)
        # sys.stderr = TextBrowserRedirector(self.textBrowser)
        # print("This will be printed to the textBrowser.")
        # print("You can redirect any print statement to the textBrowser.")
        # Reset stdout to the default console
        # sys.stdout = sys.__stdout__

    def on_click(self):
        self.subscribe.Subscriber(self.subscribe.Topic[0])

    def on_click2(self):
        self.checkBox.setChecked(False)
        self.checkBox_2.setChecked(False)
        self.checkBox_3.setChecked(False)
        self.checkBox_4.setChecked(False)
        self.checkBox_5.setChecked(False)
        self.checkBox_6.setChecked(False)
        self.checkBox_7.setChecked(False)
        self.checkBox_8.setChecked(False)
        self.checkBox_9.setChecked(False)
        self.checkBox_10.setChecked(False)
        self.checkBox_11.setChecked(False)
        self.checkBox_12.setChecked(False)
        self.checkBox_13.setChecked(False)

    def on_click3(self):
        self.subscribe.end_subscriber()
        self.subscribe.Subscriber(self.subscribe.Topic[0])

    # def startAnalysis(self, Path):
    #     global results
    #     print('Start Analysis')
    #     framelimit = [0, 100]
    #     BoundingBox = []
    #     IntensityBox = []
    #     self.subscribe.Subscriber(self.subscribe.Topic[0])

    def doCheck(self, p):
        # print(self.subscribe.Option)
        if self.checkBox.isChecked():
            self.subscribe.Option.loc[:, 'Write_CSV'] = 1
        else:
            self.subscribe.Option.loc[:, 'Write_CSV'] = 0

    def doCheck1(self, p):
        if self.checkBox_2.isChecked():
            self.subscribe.Option.loc[:, 'Diff_Facet_POD'] = 1
        else:
            self.subscribe.Option.loc[:, 'Diff_Facet_POD'] = 0

    def doCheck2(self, p):
        if self.checkBox_3.isChecked():
            self.subscribe.Option.loc[:, 'POD'] = 1
        else:
            self.subscribe.Option.loc[:, 'POD'] = 0

    def doCheck3(self, p):
        if self.checkBox_4.isChecked():
            self.subscribe.Option.loc[:, 'Precision'] = 1
        else:
            self.subscribe.Option.loc[:, 'Precision'] = 0

    def doCheck4(self, p):
        if self.checkBox_5.isChecked():
            self.subscribe.Option.loc[:, 'Diff_Facet_Angle'] = 1
        else:
            self.subscribe.Option.loc[:, 'Diff_Facet_Angle'] = 0

    def doCheck5(self, p):
        if self.checkBox_6.isChecked():
            self.subscribe.Option.loc[:, 'Noise_Number'] = 1
        else:
            self.subscribe.Option.loc[:, 'Noise_Number'] = 0

    def doCheck6(self, p):
        if self.checkBox_7.isChecked():
            self.subscribe.Option.loc[:, 'Mean_Intensity'] = 1
        else:
            self.subscribe.Option.loc[:, 'Mean_Intensity'] = 0

    def doCheck7(self, p):
        if self.checkBox_8.isChecked():
            self.subscribe.Option.loc[:, 'FOV_Resolution'] = 1
        else:
            self.subscribe.Option.loc[:, 'FOV_Resolution'] = 0


    def doCheck8(self, p):
        if self.checkBox_9.isChecked():
            self.subscribe.Option.loc[:, 'Distance'] = 1
        else:
            self.subscribe.Option.loc[:, 'Distance'] = 0


    # def doCheck9(self, p):
    #     if self.checkBox_8.isChecked():
    #         self.subscribe.Option.loc[:, 'topic'] = 1
    #     else:
    #         self.subscribe.Option.loc[:, 'topic'] = 0

    def doCheck10(self, p):
        if self.checkBox_10.isChecked():
            self.subscribe.Option.loc[:, 'Width'] = self.lineEdit.text()
        else:
            self.subscribe.Option.loc[:, 'Width'] = None

    def doCheck11(self, p):
        if self.checkBox_11.isChecked():
            self.subscribe.Option.loc[:, 'Height'] = self.lineEdit_2.text()
        else:
            self.subscribe.Option.loc[:, 'Height'] = None

    def doCheck12(self, p):
        if self.checkBox_12.isChecked():
            self.subscribe.Option.loc[:, 'frame'] = self.lineEdit_3.text()
        else:
            self.subscribe.Option.loc[:, 'frame'] = 100

    def doCheck13(self, p):
        if self.checkBox_13.isChecked():
            self.subscribe.Option.loc[:, 'Intensity'] = [self.lineEdit_4.text()]
        else:
            self.subscribe.Option.loc[:, 'Intensity'] = [None]
        print(self.subscribe.Option)

    def changeTopic(self, int):
        self.subscribe.Option.loc[:, 'Subscribe_Topic'] = int + 1

    # def StrToInt(self, list):
    #     list = [eval(i) for i in list]
    #     return list


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main = Main()
    main.show()
    sys.exit(app.exec_())

