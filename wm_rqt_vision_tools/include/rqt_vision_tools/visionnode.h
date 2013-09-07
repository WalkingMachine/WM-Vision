#ifndef VISIONNODE_H
#define VISIONNODE_H

#include <ui_visionnode.h>

#include <QFrame>
#include <map>

namespace Ui {
class VisionNode;
}

class VisionNode : public QFrame
{
    Q_OBJECT
    
public:
    explicit VisionNode(QWidget *parent = 0);
    ~VisionNode();

    void setNode(QString id, QString type = "", QString debug_type= "", std::map<std::string, std::string> dependences = std::map<std::string, std::string>(), std::map<std::string, std::string> parameters = std::map<std::string, std::string>());
    QString id();
    QString type();
    QString debug_node();
    std::map<std::string, std::string> dependences();
    std::map<std::string, std::string> parameters();

public slots:
    void AutoSizeQTableWidget();
    void AutoSizeDependencesQTableWidget();

    
private:
    Ui::VisionNode *ui;

    template <class T>
        void AutoSizeTableWidget() {
            T *table_widget = (T*)sender();

            int number_empty_cell_on_row;

            for(int i_row = 0; i_row < table_widget->rowCount(); i_row++) {  // Each row
                number_empty_cell_on_row  = 0;

                for(int i_column = 0; i_column < table_widget->columnCount(); i_column++) {  // Each column
                    if(!table_widget->item(i_row, i_column) || table_widget->item(i_row, i_column)->text().isEmpty()) {  // If empty cell
                        number_empty_cell_on_row ++;
                    }
                }

                if (i_row + 1 == table_widget->rowCount()) {                    // Ender row
                    if(number_empty_cell_on_row != table_widget->columnCount()) // Is not empty
                        table_widget->insertRow(i_row + 1);
                }
                else if(number_empty_cell_on_row == table_widget->columnCount()) {  //An other row is empty
                        table_widget->removeRow(i_row);
                }
            }
        }
};

#endif // VISIONNODE_H
