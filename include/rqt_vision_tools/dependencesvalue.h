#ifndef DEPENDENCESVALUE_H
#define DEPENDENCESVALUE_H

#include <ui_dependencesvalue.h>

#include <QWidget>

namespace Ui {
class DependencesValue;
}

class DependencesValue : public QWidget
{
    Q_OBJECT
    
public:
    explicit DependencesValue(QWidget *parent = 0);
    DependencesValue(QString value, QWidget *parent = 0);
    ~DependencesValue();
    QString text();

public slots:
    void setText(QString value);
    void setValueWidgetType(QString value);
    void ToogleBool();
    void ChangeIntegerValue(int value);
    void ChangeDoubleValue(double value);
    
private:
    Ui::DependencesValue *ui;
};

#endif // DEPENDENCESVALUE_H
