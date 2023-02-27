/********************************************************************************
** Form generated from reading UI file 'vision_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VISION_GUI_H
#define UI_VISION_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Vision_GUI
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_2;
    QScrollArea *scrollArea_2;
    QWidget *scrollAreaWidgetContents;
    QGridLayout *gridLayout;
    QGroupBox *ScaleBox;
    QGridLayout *gridLayout_5;
    QSlider *ScaleBar;
    QLabel *ScaleLabel;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents_2;
    QFormLayout *formLayout;
    QLabel *ImageViewer;
    QGroupBox *TesterBox;
    QGridLayout *gridLayout_10;
    QLabel *Carbon;
    QSpinBox *MinHole;
    QSpinBox *MinCarbon;
    QLabel *Hole;
    QSpinBox *MaxCarbon;
    QSpinBox *MaxHole;
    QLabel *Thresh;
    QSpinBox *ThreshValue;
    QGroupBox *ParameterBox;
    QGridLayout *gridLayout_4;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_7;
    QSpinBox *up2_spin;
    QLabel *label_10;
    QLabel *label_9;
    QSpinBox *low2_spin;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_6;
    QLabel *label_6;
    QLabel *label_7;
    QSpinBox *up1_spin;
    QSpinBox *low1_spin;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_8;
    QSpinBox *up3_spin;
    QLabel *label_12;
    QLabel *label_13;
    QSpinBox *low3_spin;
    QPushButton *SaveBtn;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_9;
    QLabel *label_3;
    QLabel *label_2;
    QSpinBox *over23_spin;
    QSpinBox *right_spin;
    QLabel *label_14;
    QSpinBox *left_spin;
    QSpinBox *over12_spin;
    QLabel *label_5;
    QSpinBox *height_spin;
    QLabel *label_15;
    QGroupBox *ImgChoiceBox;
    QGridLayout *gridLayout_3;
    QRadioButton *Connected_radio;
    QRadioButton *ROI1_radio;
    QRadioButton *ROI2_radio;
    QRadioButton *ROI3_radio;

    void setupUi(QMainWindow *Vision_GUI)
    {
        if (Vision_GUI->objectName().isEmpty())
            Vision_GUI->setObjectName(QString::fromUtf8("Vision_GUI"));
        Vision_GUI->resize(1253, 805);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Vision_GUI->sizePolicy().hasHeightForWidth());
        Vision_GUI->setSizePolicy(sizePolicy);
        Vision_GUI->setTabletTracking(false);
        Vision_GUI->setDockOptions(QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks);
        centralWidget = new QWidget(Vision_GUI);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout_2 = new QGridLayout(centralWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        scrollArea_2 = new QScrollArea(centralWidget);
        scrollArea_2->setObjectName(QString::fromUtf8("scrollArea_2"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(scrollArea_2->sizePolicy().hasHeightForWidth());
        scrollArea_2->setSizePolicy(sizePolicy1);
        scrollArea_2->setFrameShape(QFrame::WinPanel);
        scrollArea_2->setFrameShadow(QFrame::Sunken);
        scrollArea_2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea_2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea_2->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        scrollArea_2->setWidgetResizable(true);
        scrollArea_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 1217, 769));
        scrollAreaWidgetContents->setContextMenuPolicy(Qt::DefaultContextMenu);
        gridLayout = new QGridLayout(scrollAreaWidgetContents);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        ScaleBox = new QGroupBox(scrollAreaWidgetContents);
        ScaleBox->setObjectName(QString::fromUtf8("ScaleBox"));
        sizePolicy1.setHeightForWidth(ScaleBox->sizePolicy().hasHeightForWidth());
        ScaleBox->setSizePolicy(sizePolicy1);
        QFont font;
        font.setPointSize(8);
        ScaleBox->setFont(font);
        gridLayout_5 = new QGridLayout(ScaleBox);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        ScaleBar = new QSlider(ScaleBox);
        ScaleBar->setObjectName(QString::fromUtf8("ScaleBar"));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(ScaleBar->sizePolicy().hasHeightForWidth());
        ScaleBar->setSizePolicy(sizePolicy2);
        ScaleBar->setMinimum(1);
        ScaleBar->setMaximum(200);
        ScaleBar->setValue(50);
        ScaleBar->setOrientation(Qt::Horizontal);

        gridLayout_5->addWidget(ScaleBar, 0, 1, 1, 1);

        ScaleLabel = new QLabel(ScaleBox);
        ScaleLabel->setObjectName(QString::fromUtf8("ScaleLabel"));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(ScaleLabel->sizePolicy().hasHeightForWidth());
        ScaleLabel->setSizePolicy(sizePolicy3);

        gridLayout_5->addWidget(ScaleLabel, 0, 0, 1, 1);


        gridLayout->addWidget(ScaleBox, 2, 1, 1, 1);

        scrollArea = new QScrollArea(scrollAreaWidgetContents);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        sizePolicy1.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy1);
        scrollArea->viewport()->setProperty("cursor", QVariant(QCursor(Qt::PointingHandCursor)));
        scrollArea->setContextMenuPolicy(Qt::PreventContextMenu);
        scrollArea->setFrameShape(QFrame::Box);
        scrollArea->setFrameShadow(QFrame::Plain);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        scrollArea->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, 0, 937, 735));
        formLayout = new QFormLayout(scrollAreaWidgetContents_2);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        ImageViewer = new QLabel(scrollAreaWidgetContents_2);
        ImageViewer->setObjectName(QString::fromUtf8("ImageViewer"));
        ImageViewer->setEnabled(true);
        sizePolicy1.setHeightForWidth(ImageViewer->sizePolicy().hasHeightForWidth());
        ImageViewer->setSizePolicy(sizePolicy1);
        ImageViewer->setScaledContents(true);

        formLayout->setWidget(0, QFormLayout::LabelRole, ImageViewer);

        scrollArea->setWidget(scrollAreaWidgetContents_2);

        gridLayout->addWidget(scrollArea, 0, 0, 4, 1);

        TesterBox = new QGroupBox(scrollAreaWidgetContents);
        TesterBox->setObjectName(QString::fromUtf8("TesterBox"));
        sizePolicy1.setHeightForWidth(TesterBox->sizePolicy().hasHeightForWidth());
        TesterBox->setSizePolicy(sizePolicy1);
        TesterBox->setFont(font);
        gridLayout_10 = new QGridLayout(TesterBox);
        gridLayout_10->setSpacing(6);
        gridLayout_10->setContentsMargins(11, 11, 11, 11);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        Carbon = new QLabel(TesterBox);
        Carbon->setObjectName(QString::fromUtf8("Carbon"));
        Carbon->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(Carbon, 1, 1, 1, 1);

        MinHole = new QSpinBox(TesterBox);
        MinHole->setObjectName(QString::fromUtf8("MinHole"));
        MinHole->setMaximum(50000);

        gridLayout_10->addWidget(MinHole, 0, 0, 1, 1);

        MinCarbon = new QSpinBox(TesterBox);
        MinCarbon->setObjectName(QString::fromUtf8("MinCarbon"));
        MinCarbon->setMaximum(50000);

        gridLayout_10->addWidget(MinCarbon, 1, 0, 1, 1);

        Hole = new QLabel(TesterBox);
        Hole->setObjectName(QString::fromUtf8("Hole"));
        Hole->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(Hole, 0, 1, 1, 1);

        MaxCarbon = new QSpinBox(TesterBox);
        MaxCarbon->setObjectName(QString::fromUtf8("MaxCarbon"));
        MaxCarbon->setMaximum(50000);

        gridLayout_10->addWidget(MaxCarbon, 1, 2, 1, 1);

        MaxHole = new QSpinBox(TesterBox);
        MaxHole->setObjectName(QString::fromUtf8("MaxHole"));
        MaxHole->setMaximum(50000);

        gridLayout_10->addWidget(MaxHole, 0, 2, 1, 1);

        Thresh = new QLabel(TesterBox);
        Thresh->setObjectName(QString::fromUtf8("Thresh"));
        Thresh->setAlignment(Qt::AlignCenter);

        gridLayout_10->addWidget(Thresh, 2, 0, 1, 1);

        ThreshValue = new QSpinBox(TesterBox);
        ThreshValue->setObjectName(QString::fromUtf8("ThreshValue"));
        ThreshValue->setMaximum(50000);

        gridLayout_10->addWidget(ThreshValue, 2, 1, 1, 2);


        gridLayout->addWidget(TesterBox, 1, 1, 1, 1);

        ParameterBox = new QGroupBox(scrollAreaWidgetContents);
        ParameterBox->setObjectName(QString::fromUtf8("ParameterBox"));
        QSizePolicy sizePolicy4(QSizePolicy::Maximum, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(ParameterBox->sizePolicy().hasHeightForWidth());
        ParameterBox->setSizePolicy(sizePolicy4);
        ParameterBox->setFont(font);
        gridLayout_4 = new QGridLayout(ParameterBox);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        groupBox_2 = new QGroupBox(ParameterBox);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_7 = new QGridLayout(groupBox_2);
        gridLayout_7->setSpacing(6);
        gridLayout_7->setContentsMargins(11, 11, 11, 11);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        up2_spin = new QSpinBox(groupBox_2);
        up2_spin->setObjectName(QString::fromUtf8("up2_spin"));
        QSizePolicy sizePolicy5(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(0);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(up2_spin->sizePolicy().hasHeightForWidth());
        up2_spin->setSizePolicy(sizePolicy5);
        up2_spin->setMaximum(2000);
        up2_spin->setValue(165);

        gridLayout_7->addWidget(up2_spin, 0, 1, 1, 1);

        label_10 = new QLabel(groupBox_2);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        sizePolicy5.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy5);

        gridLayout_7->addWidget(label_10, 0, 2, 1, 1);

        label_9 = new QLabel(groupBox_2);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        sizePolicy5.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy5);

        gridLayout_7->addWidget(label_9, 0, 0, 1, 1);

        low2_spin = new QSpinBox(groupBox_2);
        low2_spin->setObjectName(QString::fromUtf8("low2_spin"));
        sizePolicy5.setHeightForWidth(low2_spin->sizePolicy().hasHeightForWidth());
        low2_spin->setSizePolicy(sizePolicy5);
        low2_spin->setMaximum(2000);
        low2_spin->setValue(1800);

        gridLayout_7->addWidget(low2_spin, 0, 3, 1, 1);


        gridLayout_4->addWidget(groupBox_2, 12, 0, 1, 1);

        groupBox = new QGroupBox(ParameterBox);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_6 = new QGridLayout(groupBox);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        sizePolicy5.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy5);

        gridLayout_6->addWidget(label_6, 0, 0, 1, 1);

        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        sizePolicy5.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy5);

        gridLayout_6->addWidget(label_7, 0, 2, 1, 1);

        up1_spin = new QSpinBox(groupBox);
        up1_spin->setObjectName(QString::fromUtf8("up1_spin"));
        sizePolicy5.setHeightForWidth(up1_spin->sizePolicy().hasHeightForWidth());
        up1_spin->setSizePolicy(sizePolicy5);
        up1_spin->setMaximum(2000);
        up1_spin->setValue(145);

        gridLayout_6->addWidget(up1_spin, 0, 1, 1, 1);

        low1_spin = new QSpinBox(groupBox);
        low1_spin->setObjectName(QString::fromUtf8("low1_spin"));
        sizePolicy5.setHeightForWidth(low1_spin->sizePolicy().hasHeightForWidth());
        low1_spin->setSizePolicy(sizePolicy5);
        low1_spin->setMaximum(2000);
        low1_spin->setValue(1800);
        low1_spin->setDisplayIntegerBase(10);

        gridLayout_6->addWidget(low1_spin, 0, 3, 1, 1);


        gridLayout_4->addWidget(groupBox, 11, 0, 1, 1);

        groupBox_3 = new QGroupBox(ParameterBox);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_8 = new QGridLayout(groupBox_3);
        gridLayout_8->setSpacing(6);
        gridLayout_8->setContentsMargins(11, 11, 11, 11);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        up3_spin = new QSpinBox(groupBox_3);
        up3_spin->setObjectName(QString::fromUtf8("up3_spin"));
        sizePolicy5.setHeightForWidth(up3_spin->sizePolicy().hasHeightForWidth());
        up3_spin->setSizePolicy(sizePolicy5);
        up3_spin->setMaximum(2000);
        up3_spin->setValue(145);

        gridLayout_8->addWidget(up3_spin, 0, 1, 1, 1);

        label_12 = new QLabel(groupBox_3);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        sizePolicy5.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy5);

        gridLayout_8->addWidget(label_12, 0, 2, 1, 1);

        label_13 = new QLabel(groupBox_3);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        sizePolicy5.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy5);

        gridLayout_8->addWidget(label_13, 0, 0, 1, 1);

        low3_spin = new QSpinBox(groupBox_3);
        low3_spin->setObjectName(QString::fromUtf8("low3_spin"));
        sizePolicy5.setHeightForWidth(low3_spin->sizePolicy().hasHeightForWidth());
        low3_spin->setSizePolicy(sizePolicy5);
        low3_spin->setMaximum(2000);
        low3_spin->setValue(1800);

        gridLayout_8->addWidget(low3_spin, 0, 3, 1, 1);


        gridLayout_4->addWidget(groupBox_3, 13, 0, 1, 1);

        SaveBtn = new QPushButton(ParameterBox);
        SaveBtn->setObjectName(QString::fromUtf8("SaveBtn"));
        sizePolicy1.setHeightForWidth(SaveBtn->sizePolicy().hasHeightForWidth());
        SaveBtn->setSizePolicy(sizePolicy1);

        gridLayout_4->addWidget(SaveBtn, 14, 0, 1, 1);

        groupBox_4 = new QGroupBox(ParameterBox);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        gridLayout_9 = new QGridLayout(groupBox_4);
        gridLayout_9->setSpacing(6);
        gridLayout_9->setContentsMargins(11, 11, 11, 11);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        label_3 = new QLabel(groupBox_4);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        sizePolicy5.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy5);

        gridLayout_9->addWidget(label_3, 1, 0, 1, 1);

        label_2 = new QLabel(groupBox_4);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy5.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy5);

        gridLayout_9->addWidget(label_2, 0, 0, 1, 1);

        over23_spin = new QSpinBox(groupBox_4);
        over23_spin->setObjectName(QString::fromUtf8("over23_spin"));
        sizePolicy5.setHeightForWidth(over23_spin->sizePolicy().hasHeightForWidth());
        over23_spin->setSizePolicy(sizePolicy5);
        over23_spin->setMaximum(2000);
        over23_spin->setValue(100);
        over23_spin->setDisplayIntegerBase(10);

        gridLayout_9->addWidget(over23_spin, 1, 1, 1, 1);

        right_spin = new QSpinBox(groupBox_4);
        right_spin->setObjectName(QString::fromUtf8("right_spin"));
        sizePolicy5.setHeightForWidth(right_spin->sizePolicy().hasHeightForWidth());
        right_spin->setSizePolicy(sizePolicy5);
        right_spin->setMaximum(2000);
        right_spin->setValue(100);

        gridLayout_9->addWidget(right_spin, 3, 1, 1, 1);

        label_14 = new QLabel(groupBox_4);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        sizePolicy5.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy5);

        gridLayout_9->addWidget(label_14, 2, 0, 1, 1);

        left_spin = new QSpinBox(groupBox_4);
        left_spin->setObjectName(QString::fromUtf8("left_spin"));
        sizePolicy5.setHeightForWidth(left_spin->sizePolicy().hasHeightForWidth());
        left_spin->setSizePolicy(sizePolicy5);
        left_spin->setMaximum(2000);
        left_spin->setValue(100);

        gridLayout_9->addWidget(left_spin, 2, 1, 1, 1);

        over12_spin = new QSpinBox(groupBox_4);
        over12_spin->setObjectName(QString::fromUtf8("over12_spin"));
        sizePolicy5.setHeightForWidth(over12_spin->sizePolicy().hasHeightForWidth());
        over12_spin->setSizePolicy(sizePolicy5);
        over12_spin->setMaximum(2000);
        over12_spin->setValue(100);
        over12_spin->setDisplayIntegerBase(10);

        gridLayout_9->addWidget(over12_spin, 0, 1, 1, 1);

        label_5 = new QLabel(groupBox_4);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        sizePolicy5.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy5);

        gridLayout_9->addWidget(label_5, 3, 0, 1, 1);

        height_spin = new QSpinBox(groupBox_4);
        height_spin->setObjectName(QString::fromUtf8("height_spin"));
        sizePolicy5.setHeightForWidth(height_spin->sizePolicy().hasHeightForWidth());
        height_spin->setSizePolicy(sizePolicy5);
        height_spin->setMaximum(2000);
        height_spin->setValue(1735);

        gridLayout_9->addWidget(height_spin, 4, 1, 1, 1);

        label_15 = new QLabel(groupBox_4);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        sizePolicy5.setHeightForWidth(label_15->sizePolicy().hasHeightForWidth());
        label_15->setSizePolicy(sizePolicy5);

        gridLayout_9->addWidget(label_15, 4, 0, 1, 1);


        gridLayout_4->addWidget(groupBox_4, 9, 0, 2, 1);


        gridLayout->addWidget(ParameterBox, 3, 1, 1, 1);

        ImgChoiceBox = new QGroupBox(scrollAreaWidgetContents);
        ImgChoiceBox->setObjectName(QString::fromUtf8("ImgChoiceBox"));
        sizePolicy1.setHeightForWidth(ImgChoiceBox->sizePolicy().hasHeightForWidth());
        ImgChoiceBox->setSizePolicy(sizePolicy1);
        ImgChoiceBox->setFont(font);
        gridLayout_3 = new QGridLayout(ImgChoiceBox);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        Connected_radio = new QRadioButton(ImgChoiceBox);
        Connected_radio->setObjectName(QString::fromUtf8("Connected_radio"));
        sizePolicy5.setHeightForWidth(Connected_radio->sizePolicy().hasHeightForWidth());
        Connected_radio->setSizePolicy(sizePolicy5);
        Connected_radio->setCheckable(true);
        Connected_radio->setChecked(false);
        Connected_radio->setAutoExclusive(true);

        gridLayout_3->addWidget(Connected_radio, 2, 0, 1, 2);

        ROI1_radio = new QRadioButton(ImgChoiceBox);
        ROI1_radio->setObjectName(QString::fromUtf8("ROI1_radio"));
        sizePolicy5.setHeightForWidth(ROI1_radio->sizePolicy().hasHeightForWidth());
        ROI1_radio->setSizePolicy(sizePolicy5);
        ROI1_radio->setCheckable(true);
        ROI1_radio->setChecked(false);
        ROI1_radio->setAutoExclusive(true);

        gridLayout_3->addWidget(ROI1_radio, 1, 0, 1, 1);

        ROI2_radio = new QRadioButton(ImgChoiceBox);
        ROI2_radio->setObjectName(QString::fromUtf8("ROI2_radio"));
        sizePolicy5.setHeightForWidth(ROI2_radio->sizePolicy().hasHeightForWidth());
        ROI2_radio->setSizePolicy(sizePolicy5);
        ROI2_radio->setCheckable(true);
        ROI2_radio->setChecked(false);
        ROI2_radio->setAutoExclusive(true);

        gridLayout_3->addWidget(ROI2_radio, 1, 1, 1, 1);

        ROI3_radio = new QRadioButton(ImgChoiceBox);
        ROI3_radio->setObjectName(QString::fromUtf8("ROI3_radio"));
        sizePolicy5.setHeightForWidth(ROI3_radio->sizePolicy().hasHeightForWidth());
        ROI3_radio->setSizePolicy(sizePolicy5);
        ROI3_radio->setCheckable(true);
        ROI3_radio->setChecked(false);
        ROI3_radio->setAutoExclusive(true);

        gridLayout_3->addWidget(ROI3_radio, 1, 2, 1, 1);


        gridLayout->addWidget(ImgChoiceBox, 0, 1, 1, 1);

        scrollArea_2->setWidget(scrollAreaWidgetContents);

        gridLayout_2->addWidget(scrollArea_2, 0, 0, 1, 1);

        Vision_GUI->setCentralWidget(centralWidget);

        retranslateUi(Vision_GUI);

        QMetaObject::connectSlotsByName(Vision_GUI);
    } // setupUi

    void retranslateUi(QMainWindow *Vision_GUI)
    {
        Vision_GUI->setWindowTitle(QApplication::translate("Vision_GUI", "Vision_GUI", nullptr));
        ScaleBox->setTitle(QApplication::translate("Vision_GUI", "Image Scale", nullptr));
        ScaleLabel->setText(QApplication::translate("Vision_GUI", "x0.50", nullptr));
#ifndef QT_NO_TOOLTIP
        scrollAreaWidgetContents_2->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        ImageViewer->setText(QString());
        TesterBox->setTitle(QApplication::translate("Vision_GUI", "Tester Parameter", nullptr));
        Carbon->setText(QApplication::translate("Vision_GUI", "Carbon", nullptr));
        Hole->setText(QApplication::translate("Vision_GUI", "Hole", nullptr));
        Thresh->setText(QApplication::translate("Vision_GUI", "Threshold", nullptr));
        ParameterBox->setTitle(QApplication::translate("Vision_GUI", "Parameters", nullptr));
        groupBox_2->setTitle(QApplication::translate("Vision_GUI", "Horizontal Boundary of 2", nullptr));
        label_10->setText(QApplication::translate("Vision_GUI", "Lower", nullptr));
        label_9->setText(QApplication::translate("Vision_GUI", "Upper", nullptr));
        groupBox->setTitle(QApplication::translate("Vision_GUI", "Horizontal Boundary of 1", nullptr));
        label_6->setText(QApplication::translate("Vision_GUI", "Upper", nullptr));
        label_7->setText(QApplication::translate("Vision_GUI", "Lower", nullptr));
        groupBox_3->setTitle(QApplication::translate("Vision_GUI", "Horizontal Boundary of 3", nullptr));
        label_12->setText(QApplication::translate("Vision_GUI", "Lower", nullptr));
        label_13->setText(QApplication::translate("Vision_GUI", "Upper", nullptr));
        SaveBtn->setText(QApplication::translate("Vision_GUI", "Save Parameters", nullptr));
        groupBox_4->setTitle(QApplication::translate("Vision_GUI", "Vertical Boundaries & Height", nullptr));
        label_3->setText(QApplication::translate("Vision_GUI", "Ovelapped Region 2 & 3", nullptr));
        label_2->setText(QApplication::translate("Vision_GUI", "Ovelapped Region 1 & 2", nullptr));
        label_14->setText(QApplication::translate("Vision_GUI", "Unnecessary Left", nullptr));
        label_5->setText(QApplication::translate("Vision_GUI", "Unnecessary Right", nullptr));
        label_15->setText(QApplication::translate("Vision_GUI", "Height of Film", nullptr));
        ImgChoiceBox->setTitle(QApplication::translate("Vision_GUI", "Choose Image", nullptr));
        Connected_radio->setText(QApplication::translate("Vision_GUI", "Connected Image", nullptr));
        ROI1_radio->setText(QApplication::translate("Vision_GUI", "ROI 1", nullptr));
        ROI2_radio->setText(QApplication::translate("Vision_GUI", "ROI 2", nullptr));
        ROI3_radio->setText(QApplication::translate("Vision_GUI", "ROI 3", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Vision_GUI: public Ui_Vision_GUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VISION_GUI_H
