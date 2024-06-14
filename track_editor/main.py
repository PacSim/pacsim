#!/usr/bin/env python3
from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import QMenu
from PyQt5.QtWidgets import QAction, QFileDialog, QLabel, QWidget, QSizePolicy, QMessageBox, QLineEdit, QFormLayout, QDialog, QPushButton
from PyQt5.QtGui import QBrush, QPen, QPixmap, QImage, QResizeEvent, QTransform, QColor, QColorConstants, QPainter, QIcon, QDoubleValidator, QRegExpValidator
from PyQt5.QtCore import Qt, QRect, QSize, Qt, QRegExp

import drawView
import guiLogic

import numpy as np


class MyWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.guiLogic = None
        self.ui = None

    def keyPressEvent(self, event):
        super().keyPressEvent(event)
        if(event.key() == Qt.Key_0):
            self.ui.changeLandmarkType(guiLogic.landmarkType.UNDEFINED, ui.coneUnknownOption)
        elif(event.key() == Qt.Key_1):
            self.ui.changeLandmarkType(guiLogic.landmarkType.BLUE, ui.coneBlueOption)
        elif(event.key() == Qt.Key_2):
            self.ui.changeLandmarkType(guiLogic.landmarkType.YELLOW, ui.coneYellowOption)
        elif(event.key() == Qt.Key_3):
            self.ui.changeLandmarkType(guiLogic.landmarkType.ORANGE, ui.coneOrangeOption)
        elif(event.key() == Qt.Key_4):
            self.ui.changeLandmarkType(guiLogic.landmarkType.BIG_ORANGE, ui.coneBigOrangeOption)
        elif(event.key() == Qt.Key_5):
            self.ui.changeLandmarkType(guiLogic.landmarkType.INVISIBLE, ui.coneInvisibleOption)
        elif(event.key() == Qt.Key_C):
            self.guiLogic.editorMode = guiLogic.editorMode.LANE_CONNECT_LEFT
            self.ui.deactiveOptionsExcept(self.ui.laneConnectOption)
            self.ui.activateLaneConnection()
        elif(event.key() == Qt.Key_T):
            self.ui.activateTimeKeepingLine()
        elif(event.key() == Qt.Key_P):
            self.ui.startPoseSettings()
        elif(event.key() == Qt.Key_G):
            self.ui.gnssSettings()

        elif(event.key() == Qt.Key_Escape):
            self.guiLogic.editorMode = guiLogic.editorMode.ADD
            self.ui.graphicsView.checkTimeKeepingLines()
            self.guiLogic.graphicsView.resetLaneConnections()
            self.guiLogic.graphicsView.addModeInit()
            self.ui.changeLandmarkType(self.guiLogic.landmarkType, self.ui.lastLandmarkOption)


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        self.MainWindow = MainWindow
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.setWindowTitle("Track editor")

        self.MainWindow.resize(800, 600)
        self.MainWindow.setWindowIcon(QIcon('icons/icon.png'))
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.graphicsView = drawView.drawView(self.centralwidget)
        self.graphicsView.setRenderHints(QtGui.QPainter.HighQualityAntialiasing|QtGui.QPainter.SmoothPixmapTransform|QtGui.QPainter.TextAntialiasing)
        self.graphicsView.setObjectName("graphicsView")
        self.horizontalLayout.addWidget(self.graphicsView)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.MainWindow.setCentralWidget(self.centralwidget)
        self.menuBar = QtWidgets.QMenuBar(self.MainWindow)
        # self.menuBar.setGeometry(QtCore.QRect(0, 0, 500, 20))
        self.menuBar.setObjectName("menuBar")
        self._createMenuBarOptions()
        self.menuFile = QMenu("&File")
        self.menuFile.addAction(self.openFileAction)
        self.menuFile.addAction(self.saveFileAction)
        self.menuBar.addMenu(self.menuFile)
        self.menuHelp = QMenu("&Help")
        self.menuHelp.addAction(self.controlsHelpAction)
        self.menuHelp.addAction(self.aboutAction)
        self.menuBar.addMenu(self.menuHelp)

        self.MainWindow.setMenuBar(self.menuBar)
        
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.label = QLabel("", self.MainWindow)
        self.label.setAlignment(Qt.AlignRight)
        self.statusbar.addWidget(self.label)
        # self.statusbar.setAlignment(Qt.AlignRight)
        # self.statusbar.showMessage("hi")
        self._creatToolbarOptions()
        self.toolBar = QtWidgets.QToolBar(self.MainWindow)
        self.toolBar.setObjectName("toolBar")
        self.toolBar.setWindowTitle("toolBar")
        self.toolBar.addAction(self.coneBlueOption)
        self.toolBar.addAction(self.coneYellowOption)
        self.toolBar.addAction(self.coneOrangeOption)
        self.toolBar.addAction(self.coneBigOrangeOption)
        # self.toolBar.addAction(self.timeKeepingOption)
        self.toolBar.addAction(self.coneInvisibleOption)
        self.toolBar.addAction(self.coneUnknownOption)
        self.toolBar.addAction(self.laneConnectOption)
        self.toolBar.insertSeparator(self.laneConnectOption)
        self.toolBar.addAction(self.timeKeepingLineOption)
        self.toolBar.insertSeparator(self.startPoseOption)
        self.toolBar.addAction(self.startPoseOption)
        self.toolBar.addAction(self.gnssSettingsOption)
        self.toolBar.addAction(self.rulesComplianceCheckAction)
        self.toolBar.insertSeparator(self.rulesComplianceCheckAction)
        for o in self.toolBar.actions():
            o.setCheckable(True)
        self.lastLandmarkOption = ui.coneBlueOption
        self.MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.widget = None

        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)
        self.guiLogic = None
        self.modeActionMap = {}
        self.modeActionMap[guiLogic.landmarkType.UNDEFINED] = self.coneUnknownOption
        self.modeActionMap[guiLogic.landmarkType.BLUE] = self.coneBlueOption
        self.modeActionMap[guiLogic.landmarkType.YELLOW] = self.coneBlueOption
        self.modeActionMap[guiLogic.landmarkType.ORANGE] = self.coneOrangeOption
        self.modeActionMap[guiLogic.landmarkType.BIG_ORANGE] = self.coneBigOrangeOption
        self.modeActionMap[guiLogic.landmarkType.INVISIBLE] = self.coneInvisibleOption

    def save_file_dialog(self):
        file = ""
        filename, ok = QFileDialog.getSaveFileName(
            self.MainWindow,
            "Select a File",
            file, 
            "Pacsim map yaml (*.yaml)"
        )
        if filename:
            self.guiLogic.writeMapFile(filename)

    def open_file_dialog(self):
        file = ""
        filename, ok = QFileDialog.getOpenFileName(
            self.MainWindow,
            "Select a File",
            file, 
            "Pacsim map yaml (*.yaml)"
        )
        if filename:
            self.graphicsView.removeAllCones()
            self.graphicsView.resetAll()
            guiLogic.readMapFile(filename)
            guiLogic.drawCones()
            self.graphicsView.scene().originCar.setTransform(self.guiLogic.startPosition, self.guiLogic.startOrientation)
            self.graphicsView.scene().updateOriginLines()
            # self.guiLogic.writeMapFile(filename)

    def _createMenuBarOptions(self):
        self.openFileAction = QAction("&Open file")
        self.openFileAction.setShortcut("Ctrl+O")
        self.openFileAction.setStatusTip('Open file')
        self.openFileAction.triggered.connect(self.open_file_dialog)

        self.saveFileAction = QAction("&Save File")
        self.saveFileAction.setShortcut("Ctrl+S")
        self.saveFileAction.setStatusTip('Save File')
        self.saveFileAction.triggered.connect(self.save_file_dialog)
        self.controlsHelpAction = QAction("&Controls")
        self.aboutAction = QAction("&About")

    def changeLandmarkType(self, t, option):
        self.goBackToAdd()
        self.guiLogic.landmarkType = t
        self.lastLandmarkOption = option
        self.deactiveOptionsExcept(option)

    def goBackToAdd(self):
        if(self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_LEFT or self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_RIGHT):
              self.guiLogic.graphicsView.resetLaneConnections()
        if(self.guiLogic.editorMode != guiLogic.editorMode.ADD):
              self.graphicsView.checkTimeKeepingLines()
              self.guiLogic.graphicsView.addModeInit()
        self.guiLogic.editorMode = guiLogic.editorMode.ADD
        self.deactiveOptionsExcept(self.modeActionMap[self.guiLogic.landmarkType])
        self.modeActionMap[self.guiLogic.landmarkType].setChecked(True)

    def deactiveOptionsExcept(self, option):
        for i in self.toolBar.actions():
            if option is not i:
              i.setChecked(False)
            else:
              i.setChecked(True)

    def activateLaneConnection(self):
      self.guiLogic.editorMode = guiLogic.editorMode.LANE_CONNECT_LEFT
      self.graphicsView.resetLaneConnections()
      self.laneConnectOption.setChecked(True)
      self.deactiveOptionsExcept(self.laneConnectOption)

    def activateTimeKeepingLine(self):
      self.guiLogic.editorMode = guiLogic.editorMode.TIMEKEEPING_START
      # self.graphicsView.resetLaneConnections()
      self.timeKeepingLineOption.setChecked(True)
      self.deactiveOptionsExcept(self.timeKeepingLineOption)

    def rulesCheck(self):
      self.rulesComplianceCheckAction.setChecked(False)
      trackLength = self.guiLogic.getTrackLength()
      minTrackWidth = self.guiLogic.getMinTrackWidth()
      minOuterDiameter = 2*self.guiLogic.getMinOuterRadius()
      maxLaneDistance = self.guiLogic.getMaxLaneDistance()
      isCompliant = (trackLength <= 500 and trackLength >= 200) and (minTrackWidth >= 3) and (abs(minOuterDiameter) >= 9) and (maxLaneDistance <= 5)
      statusString = "Rules complaint" if isCompliant else "Not rules complaint"
      stringTrackLength = "Track length: " + str(round(trackLength,2)) + "m"
      stringTrackWidth = "Minimum track width: " + str(round(minTrackWidth,2)) + "m"
      stringOuterDiameter = "Minimum outer track diameter: " + str(round(minOuterDiameter,2)) + "m"
      stringMaxLaneDistance = "Max cone distance in lane: " + str(round(maxLaneDistance,2)) + "m"
      msgBox = QMessageBox(parent=self.MainWindow)
      msgBox.setText(statusString + "\n" + stringTrackLength + "\n" + stringTrackWidth + "\n" + stringOuterDiameter + "\n" + stringMaxLaneDistance)
      msgBox.setStandardButtons(QMessageBox.Ok)
      msgBox.setWindowIcon(QIcon('icons/icon.png'))
      msgBox.setWindowTitle("Rules check")
      msgBox.exec()

    def startPoseSettings(self):
      guiLogic.graphicsView.updateCompass()
      if(self.widget):
          self.widget.close()
      self.startPoseOption.setChecked(False)
      self.widget = QDialog(parent=self.MainWindow)
      self.widget.setWindowFlags(self.widget.windowFlags() & ~Qt.WindowType.WindowContextHelpButtonHint)
      self.widget.setWindowTitle("Start pose settings")
      self.linePoseX = QLineEdit()
      self.linePoseX.setValidator(QDoubleValidator())
      self.linePoseX.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseX.setText(str(self.guiLogic.startPosition[0]))
      self.linePoseY = QLineEdit()
      self.linePoseY.setValidator(QDoubleValidator())
      self.linePoseY.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseY.setText(str(self.guiLogic.startPosition[1]))
      self.linePoseZ = QLineEdit()
      self.linePoseZ.setValidator(QDoubleValidator())
      self.linePoseZ.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseZ.setText(str(self.guiLogic.startPosition[2]))

      r2d = 180.0 / np.pi
      self.linePoseAlpha = QLineEdit()
      self.linePoseAlpha.setValidator(QDoubleValidator())
      self.linePoseAlpha.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseAlpha.setText(str(r2d*self.guiLogic.startOrientation[0]))
      self.linePoseBeta = QLineEdit()
      self.linePoseBeta.setValidator(QDoubleValidator())
      self.linePoseBeta.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseBeta.setText(str(r2d*self.guiLogic.startOrientation[1]))
      self.linePoseGamma = QLineEdit()
      self.linePoseGamma.setValidator(QDoubleValidator())
      self.linePoseGamma.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.linePoseGamma.setText(str(r2d*self.guiLogic.startOrientation[2]))

      flo = QFormLayout()
      flo.addRow("x/m",self.linePoseX)
      flo.addRow("y/m",self.linePoseY)
      flo.addRow("z/m",self.linePoseZ)

      flo.addRow("alpha/deg",self.linePoseAlpha)
      flo.addRow("beta/deg",self.linePoseBeta)
      flo.addRow("gamma/deg",self.linePoseGamma)

      button = QPushButton("Set")
      button.clicked.connect(self.saveStartPose)
      flo.addRow(button)

      self.widget.setLayout(flo)
      self.widget.show()

    def saveStartPose(self):
        anyEmpty = len(self.linePoseX.text()) == 0 and len(self.linePoseY.text()) == 0 and len(self.linePoseZ.text()) == 0
        anyEmpty = anyEmpty and len(self.linePoseAlpha.text()) == 0 and len(self.linePoseBeta.text()) == 0 and len(self.linePoseGamma.text()) == 0
        if not anyEmpty:
          d2r = np.pi / 180.0
          self.guiLogic.startPosition = np.array([float(self.linePoseX.text()), float(self.linePoseY.text()), float(self.linePoseZ.text())])
          self.guiLogic.startOrientation = d2r * np.array([float(self.linePoseAlpha.text()), float(self.linePoseBeta.text()), float(self.linePoseGamma.text())])
          self.graphicsView.scene().originCar.setTransform(self.guiLogic.startPosition, self.guiLogic.startOrientation)
          self.graphicsView.scene().updateOriginLines()
          self.widget.close()
          self.widget = None

    def gnssSettings(self):
      if(self.widget):
          self.widget.close()
      self.gnssSettingsOption.setChecked(False)
      self.widget = QDialog(parent=self.MainWindow)
      self.widget.setWindowFlags(self.widget.windowFlags() & ~Qt.WindowType.WindowContextHelpButtonHint)
      self.widget.setWindowTitle("GNSS settings")
      self.lineLatitude = QLineEdit()
      self.lineLatitude.setValidator(QDoubleValidator())
      self.lineLatitude.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.lineLatitude.setText(str(self.guiLogic.originGeodeticCoordinates[0]))
      self.lineLongitude = QLineEdit()
      self.lineLongitude.setValidator(QDoubleValidator())
      self.lineLongitude.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.lineLongitude.setText(str(self.guiLogic.originGeodeticCoordinates[1]))
      self.lineGeodeticHeight = QLineEdit()
      self.lineGeodeticHeight.setValidator(QDoubleValidator())
      self.lineGeodeticHeight.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.lineGeodeticHeight.setText(str(self.guiLogic.originGeodeticCoordinates[2]))

      r2d = 180.0 / np.pi
      self.ENURotAlpha = QLineEdit()
      self.ENURotAlpha.setValidator(QDoubleValidator())
      self.ENURotAlpha.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.ENURotAlpha.setText(str(r2d*self.guiLogic.originENURotation[0]))
      self.ENURotBeta = QLineEdit()
      self.ENURotBeta.setValidator(QDoubleValidator())
      self.ENURotBeta.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.ENURotBeta.setText(str(r2d*self.guiLogic.originENURotation[1]))
      self.ENURotGamma = QLineEdit()
      self.ENURotGamma.setValidator(QDoubleValidator())
      self.ENURotGamma.setValidator(QRegExpValidator(QRegExp("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?")))
      self.ENURotGamma.setText(str(r2d*self.guiLogic.originENURotation[2]))

      flo = QFormLayout()
      flo.addRow("Latitude/deg",self.lineLatitude)
      flo.addRow("Longitude/deg",self.lineLongitude)
      flo.addRow("Geodetic height/m",self.lineGeodeticHeight)

      flo.addRow("alpha/deg",self.ENURotAlpha)
      flo.addRow("beta/deg",self.ENURotBeta)
      flo.addRow("gamma/deg",self.ENURotGamma)

      button = QPushButton("Set")
      button.clicked.connect(self.saveGNSS)
      flo.addRow(button)

      self.widget.setLayout(flo)
      self.widget.show()

    def saveGNSS(self):
        anyEmpty = len(self.lineLatitude.text()) == 0 and len(self.lineLongitude.text()) == 0 and len(self.lineGeodeticHeight.text()) == 0
        anyEmpty = anyEmpty and len(self.ENURotAlpha.text()) == 0 and len(self.ENURotBeta.text()) == 0 and len(self.ENURotGamma.text()) == 0
        if not anyEmpty:
          d2r = np.pi / 180.0
          self.guiLogic.originGeodeticCoordinates = np.array([float(self.lineLatitude.text()), float(self.lineLongitude.text()), float(self.lineGeodeticHeight.text())])
          self.guiLogic.originENURotation = d2r * np.array([float(self.ENURotAlpha.text()), float(self.ENURotBeta.text()), float(self.ENURotGamma.text())])
          self.widget.close()
          self.graphicsView.updateCompass()
          self.widget = None

    def _creatToolbarOptions(self):
        self.coneUnknownOption = QAction()
        self.coneUnknownOption.setText("Unknown cone (0)")
        self.coneUnknownOption.setIcon(QIcon("icons/coneUnknown.png"))
        self.coneUnknownOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.UNDEFINED, self.coneUnknownOption))
        self.coneBlueOption = QAction()
        self.coneBlueOption.setText("Blue cone (2)")
        self.coneBlueOption.setIcon(QIcon("icons/coneBlue.png"))
        self.coneBlueOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.BLUE, self.coneBlueOption))
        self.coneYellowOption = QAction()
        self.coneYellowOption.setText("Yellow cone (1)")
        self.coneYellowOption.setIcon(QIcon("icons/coneYellow.png"))
        self.coneYellowOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.YELLOW, self.coneYellowOption))
        self.coneOrangeOption = QAction()
        self.coneOrangeOption.setText("Orange cone (3)")
        self.coneOrangeOption.setIcon(QIcon("icons/coneOrange.png"))
        self.coneOrangeOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.ORANGE, self.coneOrangeOption))
        self.coneBigOrangeOption = QAction()
        self.coneBigOrangeOption.setText("Big orange cone (4)")
        self.coneBigOrangeOption.setIcon(QIcon("icons/coneBigOrange.png"))
        self.coneBigOrangeOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.BIG_ORANGE, self.coneBigOrangeOption))
        self.coneInvisibleOption = QAction()
        self.coneInvisibleOption.setText("Invisible cone (5)")
        self.coneInvisibleOption.setIcon(QIcon("icons/coneInvisible.png"))
        self.coneInvisibleOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.INVISIBLE, self.coneInvisibleOption))
        self.timeKeepingOption = QAction()
        self.timeKeepingOption.setText("Timekeeping (6)")
        self.timeKeepingOption.setIcon(QIcon("icons/timeKeeping.png"))
        self.timeKeepingOption.triggered.connect(lambda: self.changeLandmarkType(guiLogic.landmarkType.TIMEKEEPING, self.timeKeepingOption))

        self.laneConnectOption = QAction()
        self.laneConnectOption.setText("Connes lanes (C)")
        self.laneConnectOption.setIcon(QIcon("icons/laneConnectIcon.png"))
        self.laneConnectOption.triggered.connect(lambda: self.activateLaneConnection())

        self.timeKeepingLineOption = QAction()
        self.timeKeepingLineOption.setText("Timekeeping (T)")
        self.timeKeepingLineOption.setIcon(QIcon("icons/timeKeeping.png"))
        self.timeKeepingLineOption.triggered.connect(lambda: self.activateTimeKeepingLine())

        self.gnssSettingsOption = QAction()
        self.gnssSettingsOption.setText("GNSS settings (G)")
        self.gnssSettingsOption.setIcon(QIcon("icons/gnss.png"))
        self.gnssSettingsOption.triggered.connect(lambda: self.gnssSettings())

        self.startPoseOption = QAction()
        self.startPoseOption.setText("Start pose (P)")
        self.startPoseOption.setIcon(QIcon("icons/start.png"))
        self.startPoseOption.triggered.connect(lambda: self.startPoseSettings())

        self.rulesComplianceCheckAction = QAction()
        self.rulesComplianceCheckAction.setText("Rules compliance check (R)")
        self.rulesComplianceCheckAction.setIcon(QIcon("icons/rulesIcon.png"))
        self.rulesComplianceCheckAction.triggered.connect(lambda: self.rulesCheck())

        # self.gnssSettingsOption.triggered.connect(lambda: self.activateTimeKeepingLine())


    def updateMousePositionDisplay(self, str):
        self.label.setText(str)

if __name__ == "__main__":
    import sys
    guiLogic = guiLogic.guiLogic()
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = MyWindow()
    MainWindow.guiLogic = guiLogic
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    ui.guiLogic = guiLogic
    MainWindow.ui = ui
    guiLogic.graphicsView = ui.graphicsView
    guiLogic.graphicsView.mainWindow = MainWindow
    guiLogic.graphicsView.guiLogic = guiLogic
    guiLogic.drawCones()
    guiLogic.graphicsView.scene().originCar.setTransform(guiLogic.startPosition, guiLogic.startOrientation)
    guiLogic.graphicsView.scene().updateOriginLines()
    MainWindow.show()
    guiLogic.graphicsView.updateCompass()
    sys.exit(app.exec_())
