#ifndef VISIONTREE_H
#define VISIONTREE_H

#include <ui_visiontree.h>

#include <QFrame>
#include <rqt_vision_tools/vision_parser.h>

namespace Ui {
class VisionTree;
}

class VisionTree : public QFrame
{
    Q_OBJECT
    
public:
    explicit VisionTree(QWidget *parent = 0);
    ~VisionTree();

    void SetTreePath(QString path);
    void SaveCFVT();

public slots:
    void SetNodeWidget(QString item_text);
    void AddNode();
    void RemoveNode();
    
private:
    Ui::VisionTree *ui;
    QString path_;
    VisionParser::VisionTree vision_tree;
};

#endif // VISIONTREE_H
