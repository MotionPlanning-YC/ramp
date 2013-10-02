#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "TrajectoryView.h"
#include "Ros.h"


namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void resizeEvent(QResizeEvent *); //execute this function when the window size changes
protected:

signals:

private:
    Ui::MainWindow *ui;
    TrajectoryView trajView;
    Ros r;
    int argc;
    char **argv;

public slots:

};

#endif // MAINWINDOW_H
