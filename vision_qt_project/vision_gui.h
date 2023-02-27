#ifndef VISION_GUI_H
#define VISION_GUI_H

#include <QMainWindow>

namespace Ui {
class Vision_GUI;
}

class Vision_GUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit Vision_GUI(QWidget *parent = 0);
    ~Vision_GUI();

private slots:
    void on_RefreshButton_clicked();

private:
    Ui::Vision_GUI *ui;
};

#endif // VISION_GUI_H
