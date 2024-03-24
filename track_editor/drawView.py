import sys
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsItem, QGraphicsRectItem, QGraphicsEllipseItem, QApplication, QGraphicsPixmapItem, QGraphicsLineItem, QRubberBand, QMenu
from PyQt5.QtGui import QBrush, QContextMenuEvent, QPen, QPixmap, QImage, QResizeEvent, QTransform, QColor, QColorConstants, QPainter, QIcon
from PyQt5.QtCore import Qt, QRect, QSize, QLine, QLineF, QEvent
from PyQt5 import QtWidgets, QtCore
from collections import OrderedDict

import numpy as np

import mapFile
import guiLogic

class CustomItem(QtWidgets.QGraphicsEllipseItem):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setFlag(self.ItemIsSelectable)
        self.setFlag(self.ItemIsMovable)
        self.setFlag(self.ItemSendsGeometryChanges)
        self.line = None
        self.isPoint = None
        self.setSelected(True)
        self.setPos(QtCore.QPointF(10, 10))
    
class frameOrigin():
    def applyTransform(self):
      a = self.rotation[0]
      b = self.rotation[1]
      c = self.rotation[2]
      Rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
      Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
      Rz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
      # matches with tf2 rotation order
      R = Rz@Ry@Rx
      T = self.translation
      self.lineX = [T + R@self.pointsAxisX[0], T + R@self.pointsAxisX[1]]
      self.lineY = [T + R@self.pointsAxisY[0], T + R@self.pointsAxisY[1]]
      self.lineZ = [T + R@self.pointsAxisZ[0], T + R@self.pointsAxisZ[1]]
    def setTransform(self, translation, rotation):
       self.translation = translation
       self.rotation = rotation
       self.applyTransform()
    def __init__(self, *args, **kwargs):
      self.pointsAxisX = [np.array([0,0,0]),np.array([1,0,0])]
      self.pointsAxisY = [np.array([0,0,0]),np.array([0,1,0])]
      self.pointsAxisZ = [np.array([0,0,0]),np.array([0,0,1])]
      self.lineX = [self.pointsAxisX[0], self.pointsAxisX[1]]
      self.lineY = [self.pointsAxisY[0], self.pointsAxisY[1]]
      self.lineZ = [self.pointsAxisZ[0], self.pointsAxisZ[1]]
      self.translation = np.array([0,0,0])
      self.rotation = np.array([0,0,0])
      self.applyTransform()

class ConeItem(QtWidgets.QGraphicsPixmapItem):
    position = np.array([0,0])
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setFlag(self.ItemIsSelectable)
        self.setFlag(self.ItemIsMovable)
        self.setFlag(self.ItemSendsGeometryChanges)
        self.line = None
        self.isPoint = None
        self.setSelected(False)
        self.setShapeMode(QtWidgets.QGraphicsPixmapItem.ShapeMode.BoundingRectShape)

    def itemChange(self, change , value):

        if change == self.ItemPositionChange and self.scene():
            newPos = value
            self.position = np.array([newPos.x(), newPos.y()])

            # self.moveLineToCenter(newPos)

        return super(ConeItem, self).itemChange(change, value)

class drawView(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.wheelEvent = self.on_scroll
        self.setScene(drawScene())
        self.zoomLevel = 1
        self.translatex = 0
        self.translatey = 0
        shiftPressed = False
        self.lastMousePos = [0,0]
        self.currentMouseButton = 0
        self.mouseReleaseEvent = self.on_click_release
        self.mousePressEvent = self.on_click
        self.releasedShiftKlick = True
        self.rubberBand = None
        self.rubberBandUse = False
        self.rubberBandUsed = False
        self.mode = guiLogic.editorMode.ADD
        self.landmarkType = guiLogic.landmarkType.BLUE
        self.setRenderHint(QPainter.Antialiasing)
        self.coneMap = {}
        self.guiLogic = None

        self.leftLineMap = OrderedDict()
        self.rightLineMap = OrderedDict()
        self.leftLines = []
        self.rightLines = []

        # self.timeKeepingMap = OrderedDict()
        self.timeKeepingLines = []

        self.setBackgroundBrush(QBrush(QColor(230, 230, 230)))
        self.mainWindow = None
        # self.setTransformationAnchor(QGraphicsView.NoAnchor)

    def getSelected(self):
        ret = []
        for i in self.items():
          if(i.isSelected()):
              ret.append(i)
        return ret


    def delteteSelected(self):
      for i in self.getSelected():
          self.removeCone(i)

    def changeSelectedType(self, type):
      selected = self.getSelected()
      for i in selected:
          posOriginal = self.coneMap[i]
          if(posOriginal[1] == guiLogic.landmarkType.TIMEKEEPING):
             continue
          pos = self.worldToPosition(self.coneMap[i][0])
          if(type == guiLogic.landmarkType.BLUE):
            posOriginal[1] = guiLogic.landmarkType.BLUE
            self.scene().changeConeType(i,type)
          elif(type == guiLogic.landmarkType.YELLOW):
            posOriginal[1] = guiLogic.landmarkType.YELLOW
            self.scene().changeConeType(i,type)
          elif(type == guiLogic.landmarkType.ORANGE):
            posOriginal[1] = guiLogic.landmarkType.ORANGE
            self.scene().changeConeType(i,type)
          elif(type == guiLogic.landmarkType.BIG_ORANGE):
            posOriginal[1] = guiLogic.landmarkType.BIG_ORANGE
            self.scene().changeConeType(i,type)
          elif(type == guiLogic.landmarkType.UNDEFINED):
            posOriginal[1] = guiLogic.landmarkType.UNDEFINED
            self.scene().changeConeType(i,type)

    def rightClickMenu(self, pos):
        # if self.selectionModel().selection().indexes():
        #   for i in self.selectionModel().selection().indexes():
        #       row, column = i.row(), i.column()
        anySelected = False
        for i in self.items():
            anySelected = anySelected or i.isSelected()
        if (anySelected):
          menu = QMenu()
          deleteAction = menu.addAction("&Delete")
          submenuType = menu.addMenu("Change color")
          blueAction = submenuType.addAction("Blue")
          yellowAction = submenuType.addAction("Yellow")
          orangeAction = submenuType.addAction("Orange")
          bigOrangeAction = submenuType.addAction("Big orange")
          unknownAction = submenuType.addAction("Unknown")
          # action = menu.exec_(self.mapToGlobal(event.pos()))
          action = menu.exec_(pos)
          if action == deleteAction:
              self.delteteSelected()
          elif action == blueAction:
              self.changeSelectedType(guiLogic.landmarkType.BLUE)
          elif action == yellowAction:
              self.changeSelectedType(guiLogic.landmarkType.YELLOW)
          elif action == orangeAction:
              self.changeSelectedType(guiLogic.landmarkType.ORANGE)
          elif action == bigOrangeAction:
              self.changeSelectedType(guiLogic.landmarkType.BIG_ORANGE)
          elif action == unknownAction:
              self.changeSelectedType(guiLogic.landmarkType.UNDEFINED)


    def on_scroll(self, event):
        if event.modifiers() == Qt.ShiftModifier:
            # zoomScale = max(1 + 1.0 * event.angleDelta().y() / 120,0.7)
            self.translatex = event.angleDelta().x()
            self.translatey = event.angleDelta().y()
        elif event.modifiers() == Qt.AltModifier:
            self.translatex = event.angleDelta().x()
        else:
            zoomScale = max(1 + 1.0 * event.angleDelta().y() / 120,0.7)
            self.zoomLevel *= zoomScale
            self.zoomLevel = max(0.08, self.zoomLevel)
            self.zoomLevel = min(300, self.zoomLevel)
        self.zoom_in(self.zoomLevel)
        self.updateCompass()

    def zoom_in(self,scale):
        scale_tr = QTransform()
        scale_tr.scale(scale, scale)
        self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - self.translatex)
        self.verticalScrollBar().setValue(self.verticalScrollBar().value() - self.translatey)
        self.translatex = 0
        self.translatey = 0

        tr = scale_tr
        self.setTransform(tr, combine=False)

    def inSquare(self, start, end, point):
        minX = min(start[0], end[0])
        minY = min(start[1], end[1])
        maxX = max(start[0], end[0])
        maxY = max(start[1], end[1])
        result = (point[0] >= minX and point[0] <= maxX and point[1] >= minY and point[1] <= maxY)
        return result

    def updateLaneLines(self):
      for i in self.leftLines:
          self.scene().removeItem(i)
      for i in self.rightLines:
          self.scene().removeItem(i)
      # this is pretty hacky, the line also update visualization when called on mouseMoveEvent (only press or release)
      for i in range(len(self.guiLogic.lanesConnectionLeft)-1):
          start = self.worldToPosition(self.guiLogic.lanesConnectionLeft[i][0])
          end = self.worldToPosition(self.guiLogic.lanesConnectionLeft[i+1][0])
          l = QLineF(start, end)
          self.leftLines[i].setLine(l)
          self.scene().addItem(self.leftLines[i])
      for i in range(len(self.guiLogic.lanesConnectionRight)-1):
          start = self.worldToPosition(self.guiLogic.lanesConnectionRight[i][0])
          end = self.worldToPosition(self.guiLogic.lanesConnectionRight[i+1][0])
          l = QLineF(start, end)
          self.rightLines[i].setLine(l)
          self.scene().addItem(self.rightLines[i])

    def updateTimeKeepingLines(self):
      for i in self.timeKeepingLines:
          self.scene().removeItem(i)
      # this is pretty hacky, the line also update visualization when called on mouseMoveEvent (only press or release)
      for i in range(len(self.guiLogic.timeKeepingGates)):
          if(len(self.guiLogic.timeKeepingGates[i]) >= 2):
            start = self.worldToPosition(self.guiLogic.timeKeepingGates[i][0][0])
            end = self.worldToPosition(self.guiLogic.timeKeepingGates[i][1][0])
            l = QLineF(start, end)
            self.timeKeepingLines[i].setLine(l)
            self.scene().addItem(self.timeKeepingLines[i])

    def on_click(self, event):
        super().mousePressEvent(event)
        position = self.mapToScene(event.pos())
        self.updateLMPositionDisplay()
        mouseButton = event.button()
        self.updateLaneLines()
        if(self.guiLogic.editorMode == guiLogic.editorMode.ADD or self.guiLogic.editorMode == guiLogic.editorMode.TIMEKEEPING_START):
          if(self.rubberBandUsed):
              self.rubberBandUsed = False
              return
          self.originRB = event.pos()
          self.rubberBandStart = self.mapToScene(event.pos())
          self.currentMouseButton = mouseButton
          if(event.modifiers() == QtCore.Qt.ControlModifier):
              self.rubberBandUse = True
              if not self.rubberBand:
                  self.rubberBand = QRubberBand(QRubberBand.Rectangle, self)
              self.rubberBand.setGeometry(QRect(self.originRB, QSize()))
              self.rubberBand.show()

          elif(event.modifiers() == QtCore.Qt.NoModifier):

              anySelected = False
              for i in self.items():
                  anySelected = anySelected or i.isSelected()
              numberUnderMouse = 0
              for i in self.items():
                  numberUnderMouse = numberUnderMouse + int(i.isUnderMouse())
              anyUnderMouse = (numberUnderMouse >= 1)
              # left
              if(mouseButton == 1):
                  if(self.guiLogic.editorMode == guiLogic.editorMode.ADD):
                    if(not anySelected):
                        if(self.guiLogic.landmarkType == guiLogic.landmarkType.UNDEFINED):
                            t = self.scene().addConeUnknown(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.UNDEFINED])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.BLUE):
                            t = self.scene().addConeLeft(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.BLUE])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.YELLOW):
                            t = self.scene().addConeRight(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.YELLOW])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.ORANGE):
                            t = self.scene().addConeOrange(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.ORANGE])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.BIG_ORANGE):
                            t = self.scene().addConeBigOrange(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.BIG_ORANGE])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.INVISIBLE):
                            t = self.scene().addConeInvisible(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.INVISIBLE])
                            self.coneMap[t] = self.guiLogic.cones[-1]
                        elif(self.guiLogic.landmarkType == guiLogic.landmarkType.TIMEKEEPING):
                            t = self.scene().addConeTimeKeeping(position)
                            self.guiLogic.cones.append([self.positionToWorld(position), guiLogic.landmarkType.TIMEKEEPING])
                            self.coneMap[t] = self.guiLogic.cones[-1]

                  elif(self.guiLogic.editorMode == guiLogic.editorMode.TIMEKEEPING_START):
                    if(not anySelected):
                      duringLine = False
                      firstLine = len(self.guiLogic.timeKeepingGates) == 0
                      if(not firstLine):
                        duringLine = len(self.guiLogic.timeKeepingGates[-1]) == 1

                      t = self.scene().addConeTimeKeeping(position)
                      cone = [self.positionToWorld(position), guiLogic.landmarkType.TIMEKEEPING]
                      self.guiLogic.cones.append(cone)
                      self.coneMap[t] = self.guiLogic.cones[-1]
                      if(duringLine):
                        self.guiLogic.timeKeepingGates[-1].append(cone)
                        start = self.worldToPosition(self.guiLogic.timeKeepingGates[-1][0][0])
                        end = self.worldToPosition(self.guiLogic.timeKeepingGates[-1][1][0])
                        lineType = 0 
                        if len(self.timeKeepingLines) == 0:
                          lineType = 1
                        # elif (len(self.timeKeepingLines) == 1):
                        #   lineType = 2
                        t =self.scene().addTimeKeepingLine(start, end, lineType)
                        self.timeKeepingLines.append(t)
                      else:
                        self.guiLogic.timeKeepingGates.append([cone])

              # right
              elif(mouseButton == 2):
                  if(anySelected):
                    self.rightClickMenu(self.mapToGlobal(event.pos()))
                  elif (numberUnderMouse == 1):
                    for i in self.items():
                        if(i.isUnderMouse() and not (i in self.scene().gridLines)):
                            self.removeCone(i)
        elif(self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_LEFT):
          clickedCones = []
          for c in self.coneMap:
            # c.setSelected(False)
            c.setFlags(c.flags() & ~c.ItemIsMovable & ~c.ItemIsSelectable)
            if(c.isUnderMouse()):
                if(len(self.leftLineMap) >= 1):
                    # TODO check for plausibility
                    plausibleConnection = self.checkPlausibilityLaneConnection(self.coneMap[c])
                    if(len(self.leftLineMap) < 3 and (c is next(iter(self.leftLineMap)))):
                       plausibleConnection = False
                    if(plausibleConnection):
                      clickedCones.append(c)
                      self.leftLineMap[c] = self.coneMap[c] 
                      self.guiLogic.lanesConnectionLeft.append(self.coneMap[c])
                      start = self.worldToPosition(self.guiLogic.lanesConnectionLeft[-2][0])
                      end = self.worldToPosition(self.guiLogic.lanesConnectionLeft[-1][0])
                      t =self.scene().addLeftConnectionLine(start, end)
                      self.leftLines.append(t)
                else:
                    clickedCones.append(c)
                    self.leftLineMap[c] = self.coneMap[c] 
                    self.guiLogic.lanesConnectionLeft.append(self.coneMap[c])

          if(len(self.leftLineMap) >= 3):
            done = (self.guiLogic.lanesConnectionLeft[0] is self.guiLogic.lanesConnectionLeft[-1])
            if(done):
              self.guiLogic.editorMode = guiLogic.editorMode.LANE_CONNECT_RIGHT

        elif(self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_RIGHT):
          clickedCones = []
          for c in self.coneMap:
            c.setFlags(c.flags() & ~c.ItemIsMovable & ~c.ItemIsSelectable)
            if(c.isUnderMouse()):
                if(len(self.rightLineMap) >= 1):
                    plausibleConnection = self.checkPlausibilityLaneConnection(self.coneMap[c])
                    if(len(self.rightLineMap) < 3 and (c is next(iter(self.rightLineMap)))):
                       plausibleConnection = False
                    if(plausibleConnection):
                      clickedCones.append(c)
                      self.rightLineMap[c] = self.coneMap[c] 
                      self.guiLogic.lanesConnectionRight.append(self.coneMap[c])
                      start = self.worldToPosition(self.guiLogic.lanesConnectionRight[-2][0])
                      end = self.worldToPosition(self.guiLogic.lanesConnectionRight[-1][0])
                      t =self.scene().addRightConnectionLine(start, end)
                      self.rightLines.append(t)
                else:
                    clickedCones.append(c)
                    self.rightLineMap[c] = self.coneMap[c] 
                    self.guiLogic.lanesConnectionRight.append(self.coneMap[c])

          if(len(self.rightLineMap) >= 3):
            done = (self.guiLogic.lanesConnectionRight[0] is self.guiLogic.lanesConnectionRight[-1])
            if done:
              self.addModeInit()


    def checkPlausibilityLaneConnection(self, c):
      alreadyInLane = False
      startIndLeft = min(1,len(self.guiLogic.lanesConnectionLeft)) if self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_LEFT else 0
      startIndRight = min(1,len(self.guiLogic.lanesConnectionRight)) if self.guiLogic.editorMode == guiLogic.editorMode.LANE_CONNECT_RIGHT else 0
      for i in range(startIndLeft, len(self.guiLogic.lanesConnectionLeft)):
          alreadyInLane = alreadyInLane or (self.guiLogic.lanesConnectionLeft[i] is c)
      for i in range(startIndRight, len(self.guiLogic.lanesConnectionRight)):
          alreadyInLane = alreadyInLane or  (self.guiLogic.lanesConnectionRight[i] is c)
      return not alreadyInLane

    def addModeInit(self):
      self.guiLogic.editorMode = guiLogic.editorMode.ADD
      self.guiLogic.landmarkType = guiLogic.landmarkType.UNDEFINED
      for c in self.coneMap:
        c.setFlag(c.ItemIsSelectable)
        c.setFlag(c.ItemIsMovable)
      self.mainWindow.ui.goBackToAdd()

    def updateLMPositionDisplay(self):
        selected = self.getSelected()
        if len(selected) == 1:
          globalPos = self.positionToWorld(selected[0])
          self.mainWindow.ui.updateMousePositionDisplay(str(globalPos[0:2]))
        else:
          self.mainWindow.ui.updateMousePositionDisplay("")


    def on_click_release(self, event):
        self.updatePositions()
        self.updateTimeKeepingLines()
        self.updateLaneLines()
        # if(self.guiLogic.editorMode == guiLogic.editorMode.ADD):

        self.updateLMPositionDisplay()
        # print(self.coneMap)
        # print(self.guiLogic.cones)
        super().mouseReleaseEvent(event)
        mouseButton = event.button()
        self.currentMouseButton = mouseButton
        if(self.rubberBandUse):
            self.rubberBandUsed = True
            self.rubberBand.hide()
            self.rubberBandEnd = self.mapToScene(event.pos())
            scene = self.scene()
            minX = min(self.rubberBandStart.x(), self.rubberBandEnd.x())
            minY = min(self.rubberBandStart.y(), self.rubberBandEnd.y())
            maxX = max(self.rubberBandStart.x(), self.rubberBandEnd.x())
            maxY = max(self.rubberBandStart.y(), self.rubberBandEnd.y())
            for i in scene.items(minX, minY, maxX-minX, maxY-minY, Qt.IntersectsItemBoundingRect, Qt.AscendingOrder):
                i.setSelected(True)
        self.rubberBandUse = False
        self.releasedShiftKlick = True
        self.updateLMPositionDisplay()
    # def mousePos()
    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        mouseButton = event.button()
        # if(event.modifiers() == QtCore.Qt.ControlModifier):
        if(self.rubberBandUse):
            self.rubberBand.setGeometry(QRect(self.originRB, event.pos()).normalized())
        mousePos = [event.pos().x(), event.pos().y()]
        if((event.modifiers() == QtCore.Qt.ShiftModifier or self.currentMouseButton == QtCore.Qt.MiddleButton) and (not self.releasedShiftKlick)):
            movement = [mousePos[0] - self.lastMousePos[0], mousePos[1] - self.lastMousePos[1]]
            self.translatex = movement[0]
            self.translatey = movement[1]
            self.zoom_in(self.zoomLevel)
            self.updateCompass()
        self.lastMousePos = mousePos
        self.releasedShiftKlick = False

    def addCone(self, cone):
        t = None
        c = QtCore.QPointF(-cone[0][0]*self.scene().pixelPerMeter, cone[0][1]*self.scene().pixelPerMeter)
        if(cone[1] == guiLogic.landmarkType.UNDEFINED):
            t = self.scene().addConeUnknown(c)
        elif(cone[1] == guiLogic.landmarkType.BLUE):
            t = self.scene().addConeLeft(c)
        elif(cone[1] == guiLogic.landmarkType.YELLOW):
            t = self.scene().addConeRight(c)
        elif(cone[1] == guiLogic.landmarkType.ORANGE):
            t = self.scene().addConeOrange(c)
        elif(cone[1] == guiLogic.landmarkType.BIG_ORANGE):
            t = self.scene().addConeBigOrange(c)
        elif(cone[1] == guiLogic.landmarkType.INVISIBLE):
            t = self.scene().addConeInvisible(c)
        elif(cone[1] == guiLogic.landmarkType.TIMEKEEPING):
            t = self.scene().addConeTimeKeeping(c)
        else:
            print("error adding cone")
            return
        self.coneMap[t] = cone
        # print(self.coneMap)
    def removeAllCones(self):
        self.guiLogic.cones = []
        for c in self.coneMap:
            self.scene().removeItem(c)
        self.coneMap = {}

    def removeCone(self, c):
        if c not in self.coneMap:
          return
        # weird way because remove() doesn't work with np.array
        if c in self.leftLineMap.keys() or c in self.rightLineMap.keys():
            self.resetLaneConnections()
        if(self.coneMap[c][1] == guiLogic.landmarkType.TIMEKEEPING):
            self.resetTimeKeepingLines()
        else:
          self.guiLogic.cones = [i for i in self.guiLogic.cones if (i is not self.coneMap[c])]
          del self.coneMap[c]
          self.scene().removeItem(c)
        # self.guiLogic.cones.remove(self.guiLogic.cones[cone])

    def positionToWorld(self, p):
        return np.array([-p.x()/self.scene().pixelPerMeter, p.y()/self.scene().pixelPerMeter, 0])

    def worldToPosition(self, p):
        return QtCore.QPointF(-p[0]*self.scene().pixelPerMeter, p[1]*self.scene().pixelPerMeter)

    def updatePositions(self):
        for i in self.coneMap:
            worldPos = self.positionToWorld(i)
            conePos = self.coneMap[i]
            np.copyto(conePos[0], worldPos)

    def resetLaneConnections(self):
        for i in self.leftLines:
            self.scene().removeItem(i)
        for i in self.rightLines:
            self.scene().removeItem(i)
        self.leftLineMap = OrderedDict()
        self.rightLineMap = OrderedDict()
        self.leftLines = []
        self.rightLines = []
        self.guiLogic.lanesConnectionLeft = []
        self.guiLogic.lanesConnectionRight = []

    def resetTimeKeepingLines(self):
        for i in self.timeKeepingLines:
            self.scene().removeItem(i)
        self.timeKeepingLines = []
        # for in self.time
        newConeMap = {}
        toRemove = []
        for i in self.coneMap:
          if(self.coneMap[i][1] != guiLogic.landmarkType.TIMEKEEPING):  
            newConeMap[i] = self.coneMap[i]
          else:
             toRemove.append(i)
        self.coneMap = newConeMap
        for i in toRemove:
           self.scene().removeItem(i)
        # self.guiLogic.cones = [i for i in self.guiLogic.cones if (i[0] is not self.coneMap[c])]
        self.guiLogic.cones = [i for i in self.guiLogic.cones if (i[1] != guiLogic.landmarkType.TIMEKEEPING)]
        # self.guiLogic.cones = [i for i in self.guiLogic.cones if (i[0] is not self.coneMap[c])]
        self.guiLogic.timeKeepingGates = []

    def checkTimeKeepingLines(self):
      plausible = True
      for i in self.guiLogic.timeKeepingGates:
         plausible = plausible and len(i) == 2
      if(not plausible):
         self.resetTimeKeepingLines()

    def resetAll(self):
      for i in self.coneMap:
        self.removeCone(i)
      self.resetLaneConnections()
      self.resetTimeKeepingLines()

    def initFromLoad(self):
      for c in self.guiLogic.cones:
        self.addCone(c)
      counter = 0
      for i in self.guiLogic.timeKeepingGates:
        lineType = 0 
        if counter == 0:
          lineType = 1
        elif counter == 1:
          lineType = 2
        counter += 1
        start = self.worldToPosition(i[0][0])
        end = self.worldToPosition(i[1][0])
        t =self.scene().addTimeKeepingLine(start, end, lineType)
        self.timeKeepingLines.append(t)
      for i in range(0, len(self.guiLogic.lanesConnectionLeft)):
        cone = [c for c in self.coneMap.keys() if (self.coneMap[c] is self.guiLogic.lanesConnectionLeft[i])]
        self.leftLineMap[cone[0]] =  self.guiLogic.lanesConnectionLeft[i]
        if(i != 0):
          start = self.worldToPosition(self.guiLogic.lanesConnectionLeft[i-1][0])
          end = self.worldToPosition(self.guiLogic.lanesConnectionLeft[i][0])
          t =self.scene().addLeftConnectionLine(start, end)
          self.leftLines.append(t)
      for i in range(0, len(self.guiLogic.lanesConnectionRight)):
        cone = [c for c in self.coneMap.keys() if (self.coneMap[c] is self.guiLogic.lanesConnectionRight[i])]
        self.rightLineMap[cone[0]] =  self.guiLogic.lanesConnectionRight[i]
        if(i != 0):
          start = self.worldToPosition(self.guiLogic.lanesConnectionRight[i-1][0])
          end = self.worldToPosition(self.guiLogic.lanesConnectionRight[i][0])
          t =self.scene().addRightConnectionLine(start, end)
          self.rightLines.append(t)

      self.updateLaneLines()

    def updateCompass(self):
      a = self.size()
      # a = self.viewport().geometry()
      relScale = 0.3
      centerPoint = relScale * np.array([64,64])
      # centerPoint = np.array([0,0])
      c = -self.guiLogic.originENURotation[2] - np.pi/2
      Rz = np.array([[np.cos(c), -np.sin(c)], [np.sin(c), np.cos(c)]])
      centerPointRotated = Rz@centerPoint
      a = self.mapToScene(int(-centerPointRotated[0]+centerPoint[0]), int(-centerPointRotated[1]+centerPoint[1]))
      self.scene().compass.setScale(relScale*1/self.zoomLevel)
      self.scene().compass.setRotation(c * 180.0 / np.pi)
      self.scene().compass.setPos(a)


class drawScene(QGraphicsScene):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # self.mode = editorMode.ADD
        # self.landmarkType = landmarkType.BLUE
        self.mousePressEvent = self.on_click
        self.mouseReleaseEvent = self.on_click_release

        self.normalConeHeight = 64
        self.bigConeHeight = 92
        self.compassHeight = 128

        self.scalingMode = Qt.TransformationMode.SmoothTransformation # Qt.TransformationMode.FastTransformation
        self.imageBlue = QImage("icons/coneBlue.png").scaledToHeight(self.normalConeHeight, mode=self.scalingMode)
        self.imageYellow = QImage("icons/coneYellow.png").scaledToHeight(self.normalConeHeight, mode=self.scalingMode)
        self.imageOrange = QImage("icons/coneOrange.png").scaledToHeight(self.normalConeHeight, mode=self.scalingMode)
        self.imageBigOrange = QImage("icons/coneBigOrange.png").scaledToHeight(self.bigConeHeight, mode=self.scalingMode)
        self.imageUnknown = QImage("icons/coneUnknown.png").scaledToHeight(self.normalConeHeight, mode=self.scalingMode)
        self.imageInvisible = QImage("icons/coneInvisible.png").scaledToHeight(self.normalConeHeight, mode=self.scalingMode)
        self.imageTimeKeeping = QImage("icons/timeKeeping.png").scaledToHeight(self.bigConeHeight, mode=self.scalingMode)
        self.imageCompass = QImage("icons/compass.png")
        self.compass = QtWidgets.QGraphicsPixmapItem(QPixmap.fromImage(self.imageCompass).scaledToHeight(self.compassHeight, mode=self.scalingMode))
        self.compass.setPos(0,0)
        self.compass.setZValue(2)
        self.addItem(self.compass)

        self.pixelPerMeter = 64
        gridPenRough = QPen(QColor(Qt.gray),1.0)
        originPen = QPen(QColor(Qt.gray),3.0)

        self.gridLines = []
        gridSize = 30
        gridCellSize = 10.0

        self.originTrack = frameOrigin()
        self.trackOriginLines = []
        self.drawOriginLines(self.originTrack, self.trackOriginLines)

        self.originCar = frameOrigin()
        self.carOriginLines = []
        self.drawOriginLines(self.originCar, self.carOriginLines)
        # self.originCar.setTransform(np.array([2,2,0]),np.zeros(3))
        # self.updateOriginLines(self.originCar)
        for i in range(-gridSize,gridSize):
            line = QGraphicsLineItem(gridCellSize*i*self.pixelPerMeter, -gridCellSize*gridSize*self.pixelPerMeter, gridCellSize*i*self.pixelPerMeter, gridCellSize*gridSize*self.pixelPerMeter)
            line.setPen(gridPenRough)
            line.setZValue(0)
            self.addItem(line)
            self.gridLines.append(line)
        for i in range(-gridSize,gridSize):
            line = QGraphicsLineItem(-gridCellSize*gridSize*self.pixelPerMeter, gridCellSize*i*self.pixelPerMeter, gridCellSize*gridSize*self.pixelPerMeter, gridCellSize*i*self.pixelPerMeter)
            line.setPen(gridPenRough)
            line.setZValue(0)
            self.addItem(line)
            self.gridLines.append(line)

        line = QGraphicsLineItem(0, -gridCellSize*i*self.pixelPerMeter, 0, gridCellSize*gridSize*self.pixelPerMeter)
        line.setPen(originPen)
        line.setZValue(0)
        self.addItem(line)

        line = QGraphicsLineItem(-gridCellSize*i*self.pixelPerMeter, 0, gridCellSize*gridSize*self.pixelPerMeter, 0)
        line.setPen(originPen)
        line.setZValue(0)
        self.addItem(line)

    def drawOriginLines(self, origin, lines):
      penX = QPen(QColor(Qt.red),5.0)
      penX.setCapStyle(Qt.FlatCap)
      lineX = QGraphicsLineItem(-origin.lineX[0][0]*self.pixelPerMeter, origin.lineX[0][1]*self.pixelPerMeter, -origin.lineX[1][0]*self.pixelPerMeter, origin.lineX[1][1]*self.pixelPerMeter)
      lineX.setPen(penX)
      lineX.setZValue(1)
      self.addItem(lineX)
      self.gridLines.append(lineX)
      lines.append(lineX)

      penY = QPen(QColor(Qt.green),5.0)
      penY.setCapStyle(Qt.FlatCap)
      lineY = QGraphicsLineItem(-origin.lineY[0][0]*self.pixelPerMeter, origin.lineY[0][1]*self.pixelPerMeter, -origin.lineY[1][0]*self.pixelPerMeter, origin.lineY[1][1]*self.pixelPerMeter)
      lineY.setPen(penY)
      lineY.setZValue(1)
      self.addItem(lineY)
      self.gridLines.append(lineY)
      lines.append(lineY)

      penZ = QPen(QColor(Qt.blue),5.0)
      penZ.setCapStyle(Qt.FlatCap)
      lineZ = QGraphicsLineItem(-origin.lineZ[0][0]*self.pixelPerMeter, origin.lineZ[0][1]*self.pixelPerMeter, -origin.lineZ[1][0]*self.pixelPerMeter, origin.lineZ[1][1]*self.pixelPerMeter)
      lineZ.setPen(penZ)
      lineZ.setZValue(1)
      self.addItem(lineZ)
      self.gridLines.append(lineZ)
      lines.append(lineZ)

    def updateOriginLines(self):
      origin = self.originCar
      lines = self.carOriginLines
      if(len(lines) == 0):
         self.drawOriginLines(origin, lines)
      else:
        lines[0].setLine(-origin.lineX[0][0]*self.pixelPerMeter, origin.lineX[0][1]*self.pixelPerMeter, -origin.lineX[1][0]*self.pixelPerMeter, origin.lineX[1][1]*self.pixelPerMeter)
        lines[1].setLine(-origin.lineY[0][0]*self.pixelPerMeter, origin.lineY[0][1]*self.pixelPerMeter, -origin.lineY[1][0]*self.pixelPerMeter, origin.lineY[1][1]*self.pixelPerMeter)
        lines[2].setLine(-origin.lineZ[0][0]*self.pixelPerMeter, origin.lineZ[0][1]*self.pixelPerMeter, -origin.lineZ[1][0]*self.pixelPerMeter, origin.lineZ[1][1]*self.pixelPerMeter)

    def addCone(self, c):
        t = ConeItem(QPixmap.fromImage(self.imageUnknown))
        t.setOffset(-8, -8)
        t.setPos(-c[0][0]*self.pixelPerMeter, c[0][1]*self.pixelPerMeter)
        self.addItem(t)
        return t

    def on_click(self, event):
        super().mousePressEvent(event)

    def on_click_release(self, event):
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)

    def changeConeType(self, c, type):
      if(type == guiLogic.landmarkType.UNDEFINED):
        c.setPixmap(QPixmap.fromImage(self.imageUnknown))
      elif(type == guiLogic.landmarkType.BLUE):
        c.setPixmap(QPixmap.fromImage(self.imageBlue))
      elif(type == guiLogic.landmarkType.YELLOW):
        c.setPixmap(QPixmap.fromImage(self.imageYellow))
      elif(type == guiLogic.landmarkType.ORANGE):
        c.setPixmap(QPixmap.fromImage(self.imageOrange))
      elif(type == guiLogic.landmarkType.BIG_ORANGE):
        c.setPixmap(QPixmap.fromImage(self.imageBigOrange))
      elif(type == guiLogic.landmarkType.INVISIBLE):
        c.setPixmap(QPixmap.fromImage(self.imageInvisible))

    def addConeUnknown(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageUnknown))
        t.setOffset(-0.5*self.imageUnknown.size().width(), -0.5*self.imageUnknown.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t

    def addConeInvisible(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageInvisible))
        t.setOffset(-0.5*self.imageInvisible.size().width(), -0.5*self.imageInvisible.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t

    def addConeLeft(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageBlue))
        t.setOffset(-0.5*self.imageBlue.size().width(), -0.5*self.imageBlue.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t

    def addConeRight(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageYellow))
        t.setOffset(-0.5*self.imageYellow.size().width(), -0.5*self.imageYellow.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t

    def addConeOrange(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageOrange))
        t.setOffset(-0.5*self.imageOrange.size().width(), -0.5*self.imageOrange.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t
    
    def addConeBigOrange(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageBigOrange))
        t.setOffset(-0.5*self.imageBigOrange.size().width(), -0.5*self.imageBigOrange.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t
    
    def addConeTimeKeeping(self, position):    
        t = ConeItem(QPixmap.fromImage(self.imageTimeKeeping))
        t.setOffset(-0.5*self.imageTimeKeeping.size().width(), -0.5*self.imageTimeKeeping.size().height())
        t.setPos(position)
        t.setZValue(2)
        self.addItem(t)
        return t
    
    def addLeftConnectionLine(self, start, end):
        l = QLineF(start, end)
        pen = QPen(QColor(Qt.blue),4.0)
        t = self.addLine(l, pen)
        return t

    def addRightConnectionLine(self, start, end):
        l = QLineF(start, end)
        pen = QPen(QColor(Qt.darkYellow),4.0)
        t = self.addLine(l, pen)
        return t
    
    def addTimeKeepingLine(self, start, end, type):
        l = QLineF(start, end)
        pen = QPen(QColor(Qt.magenta),5.0)
        if(type == 1):
            pen = QPen(QColor(Qt.green),5.0, Qt.DashLine)
        elif(type == 2):
            pen = QPen(QColor(Qt.red),5.0, Qt.DashLine)
        t = self.addLine(l, pen)
        return t
    def zoom_in(self,scale):
        scale_tr = QTransform()
        scale_tr.scale(scale, scale)
        scale_tr.translate(self.translatex, 0)

        tr = scale_tr
        self.views()[0].setTransform(tr, combine=False)

    def zoom_out(self, scale):
        scale_tr = QTransform()
        scale_tr.scale(scale, scale)
        scale_tr.translate(self.translatex, 0)

        scale_inverted, invertible = scale_tr.inverted()

        if invertible:
            tr = scale_inverted
            self.views()[0].setTransform(tr, combine=False)