# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'vision_gui.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Vision_GUI(object):
    def setupUi(self, Vision_GUI):
        Vision_GUI.setObjectName("Vision_GUI")
        Vision_GUI.resize(1253, 805)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Vision_GUI.sizePolicy().hasHeightForWidth())
        Vision_GUI.setSizePolicy(sizePolicy)
        Vision_GUI.setTabletTracking(False)
        Vision_GUI.setDockOptions(QtWidgets.QMainWindow.AllowTabbedDocks|QtWidgets.QMainWindow.AnimatedDocks)
        self.centralWidget = QtWidgets.QWidget(Vision_GUI)
        self.centralWidget.setObjectName("centralWidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralWidget)
        self.gridLayout_2.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_2.setSpacing(6)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.scrollArea_2 = QtWidgets.QScrollArea(self.centralWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea_2.sizePolicy().hasHeightForWidth())
        self.scrollArea_2.setSizePolicy(sizePolicy)
        self.scrollArea_2.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.scrollArea_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.scrollArea_2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollArea_2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollArea_2.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.scrollArea_2.setWidgetResizable(True)
        self.scrollArea_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignTop)
        self.scrollArea_2.setObjectName("scrollArea_2")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 1217, 769))
        self.scrollAreaWidgetContents.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.gridLayout = QtWidgets.QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout.setContentsMargins(11, 11, 11, 11)
        self.gridLayout.setSpacing(6)
        self.gridLayout.setObjectName("gridLayout")
        self.ScaleBox = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ScaleBox.sizePolicy().hasHeightForWidth())
        self.ScaleBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(8)
        self.ScaleBox.setFont(font)
        self.ScaleBox.setObjectName("ScaleBox")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.ScaleBox)
        self.gridLayout_5.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_5.setSpacing(6)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.ScaleBar = QtWidgets.QSlider(self.ScaleBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ScaleBar.sizePolicy().hasHeightForWidth())
        self.ScaleBar.setSizePolicy(sizePolicy)
        self.ScaleBar.setMinimum(1)
        self.ScaleBar.setMaximum(200)
        self.ScaleBar.setProperty("value", 50)
        self.ScaleBar.setOrientation(QtCore.Qt.Horizontal)
        self.ScaleBar.setObjectName("ScaleBar")
        self.gridLayout_5.addWidget(self.ScaleBar, 0, 1, 1, 1)
        self.ScaleLabel = QtWidgets.QLabel(self.ScaleBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ScaleLabel.sizePolicy().hasHeightForWidth())
        self.ScaleLabel.setSizePolicy(sizePolicy)
        self.ScaleLabel.setObjectName("ScaleLabel")
        self.gridLayout_5.addWidget(self.ScaleLabel, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.ScaleBox, 2, 1, 1, 1)
        self.scrollArea = QtWidgets.QScrollArea(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea.sizePolicy().hasHeightForWidth())
        self.scrollArea.setSizePolicy(sizePolicy)
        self.scrollArea.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.scrollArea.setContextMenuPolicy(QtCore.Qt.PreventContextMenu)
        self.scrollArea.setFrameShape(QtWidgets.QFrame.Box)
        self.scrollArea.setFrameShadow(QtWidgets.QFrame.Plain)
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOn)
        self.scrollArea.setSizeAdjustPolicy(QtWidgets.QAbstractScrollArea.AdjustToContents)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents_2 = QtWidgets.QWidget()
        self.scrollAreaWidgetContents_2.setGeometry(QtCore.QRect(0, 0, 937, 735))
        self.scrollAreaWidgetContents_2.setToolTip("")
        self.scrollAreaWidgetContents_2.setObjectName("scrollAreaWidgetContents_2")
        self.formLayout = QtWidgets.QFormLayout(self.scrollAreaWidgetContents_2)
        self.formLayout.setContentsMargins(11, 11, 11, 11)
        self.formLayout.setSpacing(6)
        self.formLayout.setObjectName("formLayout")
        self.ImageViewer = QtWidgets.QLabel(self.scrollAreaWidgetContents_2)
        self.ImageViewer.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ImageViewer.sizePolicy().hasHeightForWidth())
        self.ImageViewer.setSizePolicy(sizePolicy)
        self.ImageViewer.setText("")
        self.ImageViewer.setScaledContents(True)
        self.ImageViewer.setObjectName("ImageViewer")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.ImageViewer)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents_2)
        self.gridLayout.addWidget(self.scrollArea, 0, 0, 4, 1)
        self.TesterBox = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.TesterBox.sizePolicy().hasHeightForWidth())
        self.TesterBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(8)
        self.TesterBox.setFont(font)
        self.TesterBox.setObjectName("TesterBox")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.TesterBox)
        self.gridLayout_10.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_10.setSpacing(6)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.Carbon = QtWidgets.QLabel(self.TesterBox)
        self.Carbon.setAlignment(QtCore.Qt.AlignCenter)
        self.Carbon.setObjectName("Carbon")
        self.gridLayout_10.addWidget(self.Carbon, 1, 1, 1, 1)
        self.MinHole = QtWidgets.QSpinBox(self.TesterBox)
        self.MinHole.setMaximum(50000)
        self.MinHole.setObjectName("MinHole")
        self.gridLayout_10.addWidget(self.MinHole, 0, 0, 1, 1)
        self.MinCarbon = QtWidgets.QSpinBox(self.TesterBox)
        self.MinCarbon.setMaximum(50000)
        self.MinCarbon.setObjectName("MinCarbon")
        self.gridLayout_10.addWidget(self.MinCarbon, 1, 0, 1, 1)
        self.Hole = QtWidgets.QLabel(self.TesterBox)
        self.Hole.setAlignment(QtCore.Qt.AlignCenter)
        self.Hole.setObjectName("Hole")
        self.gridLayout_10.addWidget(self.Hole, 0, 1, 1, 1)
        self.MaxCarbon = QtWidgets.QSpinBox(self.TesterBox)
        self.MaxCarbon.setMaximum(50000)
        self.MaxCarbon.setObjectName("MaxCarbon")
        self.gridLayout_10.addWidget(self.MaxCarbon, 1, 2, 1, 1)
        self.MaxHole = QtWidgets.QSpinBox(self.TesterBox)
        self.MaxHole.setMaximum(50000)
        self.MaxHole.setObjectName("MaxHole")
        self.gridLayout_10.addWidget(self.MaxHole, 0, 2, 1, 1)
        self.Thresh = QtWidgets.QLabel(self.TesterBox)
        self.Thresh.setAlignment(QtCore.Qt.AlignCenter)
        self.Thresh.setObjectName("Thresh")
        self.gridLayout_10.addWidget(self.Thresh, 2, 0, 1, 1)
        self.ThreshValue = QtWidgets.QSpinBox(self.TesterBox)
        self.ThreshValue.setMaximum(50000)
        self.ThreshValue.setObjectName("ThreshValue")
        self.gridLayout_10.addWidget(self.ThreshValue, 2, 1, 1, 2)
        self.gridLayout.addWidget(self.TesterBox, 1, 1, 1, 1)
        self.ParameterBox = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ParameterBox.sizePolicy().hasHeightForWidth())
        self.ParameterBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(8)
        self.ParameterBox.setFont(font)
        self.ParameterBox.setObjectName("ParameterBox")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.ParameterBox)
        self.gridLayout_4.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_4.setSpacing(6)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.groupBox_2 = QtWidgets.QGroupBox(self.ParameterBox)
        self.groupBox_2.setObjectName("groupBox_2")
        self.gridLayout_7 = QtWidgets.QGridLayout(self.groupBox_2)
        self.gridLayout_7.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_7.setSpacing(6)
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.up2_spin = QtWidgets.QSpinBox(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.up2_spin.sizePolicy().hasHeightForWidth())
        self.up2_spin.setSizePolicy(sizePolicy)
        self.up2_spin.setMaximum(2000)
        self.up2_spin.setProperty("value", 165)
        self.up2_spin.setObjectName("up2_spin")
        self.gridLayout_7.addWidget(self.up2_spin, 0, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_10.sizePolicy().hasHeightForWidth())
        self.label_10.setSizePolicy(sizePolicy)
        self.label_10.setObjectName("label_10")
        self.gridLayout_7.addWidget(self.label_10, 0, 2, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_9.sizePolicy().hasHeightForWidth())
        self.label_9.setSizePolicy(sizePolicy)
        self.label_9.setObjectName("label_9")
        self.gridLayout_7.addWidget(self.label_9, 0, 0, 1, 1)
        self.low2_spin = QtWidgets.QSpinBox(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.low2_spin.sizePolicy().hasHeightForWidth())
        self.low2_spin.setSizePolicy(sizePolicy)
        self.low2_spin.setMaximum(2000)
        self.low2_spin.setProperty("value", 1800)
        self.low2_spin.setObjectName("low2_spin")
        self.gridLayout_7.addWidget(self.low2_spin, 0, 3, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_2, 12, 0, 1, 1)
        self.groupBox = QtWidgets.QGroupBox(self.ParameterBox)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_6.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_6.setSpacing(6)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.label_6 = QtWidgets.QLabel(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy)
        self.label_6.setObjectName("label_6")
        self.gridLayout_6.addWidget(self.label_6, 0, 0, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy)
        self.label_7.setObjectName("label_7")
        self.gridLayout_6.addWidget(self.label_7, 0, 2, 1, 1)
        self.up1_spin = QtWidgets.QSpinBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.up1_spin.sizePolicy().hasHeightForWidth())
        self.up1_spin.setSizePolicy(sizePolicy)
        self.up1_spin.setMaximum(2000)
        self.up1_spin.setProperty("value", 145)
        self.up1_spin.setObjectName("up1_spin")
        self.gridLayout_6.addWidget(self.up1_spin, 0, 1, 1, 1)
        self.low1_spin = QtWidgets.QSpinBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.low1_spin.sizePolicy().hasHeightForWidth())
        self.low1_spin.setSizePolicy(sizePolicy)
        self.low1_spin.setMaximum(2000)
        self.low1_spin.setProperty("value", 1800)
        self.low1_spin.setDisplayIntegerBase(10)
        self.low1_spin.setObjectName("low1_spin")
        self.gridLayout_6.addWidget(self.low1_spin, 0, 3, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox, 11, 0, 1, 1)
        self.groupBox_3 = QtWidgets.QGroupBox(self.ParameterBox)
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.groupBox_3)
        self.gridLayout_8.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_8.setSpacing(6)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.up3_spin = QtWidgets.QSpinBox(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.up3_spin.sizePolicy().hasHeightForWidth())
        self.up3_spin.setSizePolicy(sizePolicy)
        self.up3_spin.setMaximum(2000)
        self.up3_spin.setProperty("value", 145)
        self.up3_spin.setObjectName("up3_spin")
        self.gridLayout_8.addWidget(self.up3_spin, 0, 1, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy)
        self.label_12.setObjectName("label_12")
        self.gridLayout_8.addWidget(self.label_12, 0, 2, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_13.sizePolicy().hasHeightForWidth())
        self.label_13.setSizePolicy(sizePolicy)
        self.label_13.setObjectName("label_13")
        self.gridLayout_8.addWidget(self.label_13, 0, 0, 1, 1)
        self.low3_spin = QtWidgets.QSpinBox(self.groupBox_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.low3_spin.sizePolicy().hasHeightForWidth())
        self.low3_spin.setSizePolicy(sizePolicy)
        self.low3_spin.setMaximum(2000)
        self.low3_spin.setProperty("value", 1800)
        self.low3_spin.setObjectName("low3_spin")
        self.gridLayout_8.addWidget(self.low3_spin, 0, 3, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_3, 13, 0, 1, 1)
        self.SaveBtn = QtWidgets.QPushButton(self.ParameterBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SaveBtn.sizePolicy().hasHeightForWidth())
        self.SaveBtn.setSizePolicy(sizePolicy)
        self.SaveBtn.setObjectName("SaveBtn")
        self.gridLayout_4.addWidget(self.SaveBtn, 14, 0, 1, 1)
        self.groupBox_4 = QtWidgets.QGroupBox(self.ParameterBox)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout_9.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_9.setSpacing(6)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.label_3 = QtWidgets.QLabel(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        self.label_3.setObjectName("label_3")
        self.gridLayout_9.addWidget(self.label_3, 1, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setObjectName("label_2")
        self.gridLayout_9.addWidget(self.label_2, 0, 0, 1, 1)
        self.over23_spin = QtWidgets.QSpinBox(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.over23_spin.sizePolicy().hasHeightForWidth())
        self.over23_spin.setSizePolicy(sizePolicy)
        self.over23_spin.setMaximum(2000)
        self.over23_spin.setProperty("value", 100)
        self.over23_spin.setDisplayIntegerBase(10)
        self.over23_spin.setObjectName("over23_spin")
        self.gridLayout_9.addWidget(self.over23_spin, 1, 1, 1, 1)
        self.right_spin = QtWidgets.QSpinBox(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.right_spin.sizePolicy().hasHeightForWidth())
        self.right_spin.setSizePolicy(sizePolicy)
        self.right_spin.setMaximum(2000)
        self.right_spin.setProperty("value", 100)
        self.right_spin.setObjectName("right_spin")
        self.gridLayout_9.addWidget(self.right_spin, 3, 1, 1, 1)
        self.label_14 = QtWidgets.QLabel(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_14.sizePolicy().hasHeightForWidth())
        self.label_14.setSizePolicy(sizePolicy)
        self.label_14.setObjectName("label_14")
        self.gridLayout_9.addWidget(self.label_14, 2, 0, 1, 1)
        self.left_spin = QtWidgets.QSpinBox(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.left_spin.sizePolicy().hasHeightForWidth())
        self.left_spin.setSizePolicy(sizePolicy)
        self.left_spin.setMaximum(2000)
        self.left_spin.setProperty("value", 100)
        self.left_spin.setObjectName("left_spin")
        self.gridLayout_9.addWidget(self.left_spin, 2, 1, 1, 1)
        self.over12_spin = QtWidgets.QSpinBox(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.over12_spin.sizePolicy().hasHeightForWidth())
        self.over12_spin.setSizePolicy(sizePolicy)
        self.over12_spin.setMaximum(2000)
        self.over12_spin.setProperty("value", 100)
        self.over12_spin.setDisplayIntegerBase(10)
        self.over12_spin.setObjectName("over12_spin")
        self.gridLayout_9.addWidget(self.over12_spin, 0, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        self.label_5.setObjectName("label_5")
        self.gridLayout_9.addWidget(self.label_5, 3, 0, 1, 1)
        self.height_spin = QtWidgets.QSpinBox(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.height_spin.sizePolicy().hasHeightForWidth())
        self.height_spin.setSizePolicy(sizePolicy)
        self.height_spin.setMaximum(2000)
        self.height_spin.setProperty("value", 1735)
        self.height_spin.setObjectName("height_spin")
        self.gridLayout_9.addWidget(self.height_spin, 4, 1, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.groupBox_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_15.sizePolicy().hasHeightForWidth())
        self.label_15.setSizePolicy(sizePolicy)
        self.label_15.setObjectName("label_15")
        self.gridLayout_9.addWidget(self.label_15, 4, 0, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_4, 9, 0, 2, 1)
        self.gridLayout.addWidget(self.ParameterBox, 3, 1, 1, 1)
        self.ImgChoiceBox = QtWidgets.QGroupBox(self.scrollAreaWidgetContents)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ImgChoiceBox.sizePolicy().hasHeightForWidth())
        self.ImgChoiceBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(8)
        self.ImgChoiceBox.setFont(font)
        self.ImgChoiceBox.setObjectName("ImgChoiceBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.ImgChoiceBox)
        self.gridLayout_3.setContentsMargins(11, 11, 11, 11)
        self.gridLayout_3.setSpacing(6)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.Connected_radio = QtWidgets.QRadioButton(self.ImgChoiceBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Connected_radio.sizePolicy().hasHeightForWidth())
        self.Connected_radio.setSizePolicy(sizePolicy)
        self.Connected_radio.setCheckable(True)
        self.Connected_radio.setChecked(False)
        self.Connected_radio.setAutoExclusive(True)
        self.Connected_radio.setObjectName("Connected_radio")
        self.gridLayout_3.addWidget(self.Connected_radio, 2, 0, 1, 2)
        self.ROI1_radio = QtWidgets.QRadioButton(self.ImgChoiceBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ROI1_radio.sizePolicy().hasHeightForWidth())
        self.ROI1_radio.setSizePolicy(sizePolicy)
        self.ROI1_radio.setCheckable(True)
        self.ROI1_radio.setChecked(False)
        self.ROI1_radio.setAutoExclusive(True)
        self.ROI1_radio.setObjectName("ROI1_radio")
        self.gridLayout_3.addWidget(self.ROI1_radio, 1, 0, 1, 1)
        self.ROI2_radio = QtWidgets.QRadioButton(self.ImgChoiceBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ROI2_radio.sizePolicy().hasHeightForWidth())
        self.ROI2_radio.setSizePolicy(sizePolicy)
        self.ROI2_radio.setCheckable(True)
        self.ROI2_radio.setChecked(False)
        self.ROI2_radio.setAutoExclusive(True)
        self.ROI2_radio.setObjectName("ROI2_radio")
        self.gridLayout_3.addWidget(self.ROI2_radio, 1, 1, 1, 1)
        self.ROI3_radio = QtWidgets.QRadioButton(self.ImgChoiceBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ROI3_radio.sizePolicy().hasHeightForWidth())
        self.ROI3_radio.setSizePolicy(sizePolicy)
        self.ROI3_radio.setCheckable(True)
        self.ROI3_radio.setChecked(False)
        self.ROI3_radio.setAutoExclusive(True)
        self.ROI3_radio.setObjectName("ROI3_radio")
        self.gridLayout_3.addWidget(self.ROI3_radio, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.ImgChoiceBox, 0, 1, 1, 1)
        self.scrollArea_2.setWidget(self.scrollAreaWidgetContents)
        self.gridLayout_2.addWidget(self.scrollArea_2, 0, 0, 1, 1)
        Vision_GUI.setCentralWidget(self.centralWidget)

        self.retranslateUi(Vision_GUI)
        QtCore.QMetaObject.connectSlotsByName(Vision_GUI)

    def retranslateUi(self, Vision_GUI):
        _translate = QtCore.QCoreApplication.translate
        Vision_GUI.setWindowTitle(_translate("Vision_GUI", "Vision_GUI"))
        self.ScaleBox.setTitle(_translate("Vision_GUI", "Image Scale"))
        self.ScaleLabel.setText(_translate("Vision_GUI", "x0.50"))
        self.TesterBox.setTitle(_translate("Vision_GUI", "Tester Parameter"))
        self.Carbon.setText(_translate("Vision_GUI", "Carbon"))
        self.Hole.setText(_translate("Vision_GUI", "Hole"))
        self.Thresh.setText(_translate("Vision_GUI", "Threshold"))
        self.ParameterBox.setTitle(_translate("Vision_GUI", "Parameters"))
        self.groupBox_2.setTitle(_translate("Vision_GUI", "Horizontal Boundary of 2"))
        self.label_10.setText(_translate("Vision_GUI", "Lower"))
        self.label_9.setText(_translate("Vision_GUI", "Upper"))
        self.groupBox.setTitle(_translate("Vision_GUI", "Horizontal Boundary of 1"))
        self.label_6.setText(_translate("Vision_GUI", "Upper"))
        self.label_7.setText(_translate("Vision_GUI", "Lower"))
        self.groupBox_3.setTitle(_translate("Vision_GUI", "Horizontal Boundary of 3"))
        self.label_12.setText(_translate("Vision_GUI", "Lower"))
        self.label_13.setText(_translate("Vision_GUI", "Upper"))
        self.SaveBtn.setText(_translate("Vision_GUI", "Save Parameters"))
        self.groupBox_4.setTitle(_translate("Vision_GUI", "Vertical Boundaries & Height"))
        self.label_3.setText(_translate("Vision_GUI", "Ovelapped Region 2 & 3"))
        self.label_2.setText(_translate("Vision_GUI", "Ovelapped Region 1 & 2"))
        self.label_14.setText(_translate("Vision_GUI", "Unnecessary Left"))
        self.label_5.setText(_translate("Vision_GUI", "Unnecessary Right"))
        self.label_15.setText(_translate("Vision_GUI", "Height of Film"))
        self.ImgChoiceBox.setTitle(_translate("Vision_GUI", "Choose Image"))
        self.Connected_radio.setText(_translate("Vision_GUI", "Connected Image"))
        self.ROI1_radio.setText(_translate("Vision_GUI", "ROI 1"))
        self.ROI2_radio.setText(_translate("Vision_GUI", "ROI 2"))
        self.ROI3_radio.setText(_translate("Vision_GUI", "ROI 3"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Vision_GUI = QtWidgets.QMainWindow()
    ui = Ui_Vision_GUI()
    ui.setupUi(Vision_GUI)
    Vision_GUI.show()
    sys.exit(app.exec_())
