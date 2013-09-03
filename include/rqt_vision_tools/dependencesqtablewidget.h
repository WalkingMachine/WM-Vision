#ifndef DEPENDENCESQTABLEWIDGET_H
#define DEPENDENCESQTABLEWIDGET_H

#include <QTableWidget>

class DependencesQTableWidget : public QTableWidget
{
    Q_OBJECT
public:
    explicit DependencesQTableWidget(QWidget *parent = 0);
    void insertRow(int row);
    
signals:
    
public slots:
    
};

#endif // DEPENDENCESQTABLEWIDGET_H
