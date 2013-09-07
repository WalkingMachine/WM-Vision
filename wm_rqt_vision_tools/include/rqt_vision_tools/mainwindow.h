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
    void LoadCFVTPath(QString object);
    void StartVisionTree();
    void StopVisionTree();
    void RestartVisionTree();
    
private:
    Ui::MainWindow *ui;
    VisionParser::CFVTPathMap catalog;

    void SaveCFVT();
};

#endif // MAINWINDOW_H
