#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ui_mainwindow.h>

#include <QWidget>
#include <string>
#include <map>
#include <rqt_vision_tools/vision_parser.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void LoadCatalog();
    void LoadObject(QString action);
    void LoadCFVFPath(QString object);
    void StartVisionFlow();
    void StopVisionFlow();
    void RestartVisionFlow();
    
private:
    Ui::MainWindow *ui;
    VisionParser::CFVTPathMap catalog;

    void SaveCFVF();
};

#endif // MAINWINDOW_H
