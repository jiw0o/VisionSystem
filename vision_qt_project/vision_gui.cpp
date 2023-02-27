#include "vision_gui.h"
#include "ui_vision_gui.h"

Vision_GUI::Vision_GUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Vision_GUI)
{
    ui->setupUi(this);
}

Vision_GUI::~Vision_GUI()
{
    delete ui;
}

void Vision_GUI::on_RefreshButton_clicked()
{

}
