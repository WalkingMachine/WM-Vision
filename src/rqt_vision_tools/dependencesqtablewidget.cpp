#include <rqt_vision_tools/dependencesqtablewidget.h> //#include "dependencesqtablewidget.h"

#include <rqt_vision_tools/dependencesvalue.h> //#include "dependencesvalue.h"

#include <QPushButton>

DependencesQTableWidget::DependencesQTableWidget(QWidget *parent) :
    QTableWidget(parent) {
}

void DependencesQTableWidget::insertRow(int row) {
    ((QTableWidget*)this)->insertRow(row);
    setCellWidget(row, 1, new DependencesValue(this));
}
