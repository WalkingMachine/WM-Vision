#include <rqt_vision_tools/visionnode.h>

#include <rqt_vision_tools/dependencesvalue.h>
#include <iostream>
#include <QPushButton>
#include <QMimeData>

VisionNode::VisionNode(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::VisionNode)
{
    ui->setupUi(this);
    ui->parameters_table_widget->insertRow(0);
    connect(ui->dependences_table_widget, SIGNAL(cellChanged(int,int)), this, SLOT(AutoSizeQTableWidget()));
    connect(ui->parameters_table_widget, SIGNAL(cellChanged(int,int)), this, SLOT(AutoSizeDependencesQTableWidget()));
}

VisionNode::~VisionNode()
{
    delete ui;
}

QString VisionNode::id()
{
    return ui->id_line_edit->text();
}

QString VisionNode::type() {
    return ui->type_line_edit->text();
}

QString VisionNode::debug_node() {
    return ui->debug_type_line_edit->text();
}

std::map<std::string, std::string> VisionNode::dependences() {
	std::map<std::string, std::string> dependences;

	for(int i = 0; i < ui->dependences_table_widget->rowCount(); i++) {
		if(ui->dependences_table_widget->item(i, 0) && !ui->dependences_table_widget->item(i, 0)->text().isEmpty()) {  // If not empty cell column 0
			if(ui->dependences_table_widget->item(i, 1) && !ui->dependences_table_widget->item(i, 1)->text().isEmpty()) {  // If not empty cell column 1
				dependences.insert(std::pair<std::string, std::string>(
				    		           ui->dependences_table_widget->item(i,0)->text().toStdString(),
				                       ui->dependences_table_widget->item(i,1)->text().toStdString()));
			}
		}
	}

    return dependences;
}

std::map<std::string, std::string> VisionNode::parameters() {
	std::map<std::string, std::string> parameters;

	for(int i = 0; i < ui->parameters_table_widget->rowCount(); i++) {
		if(ui->parameters_table_widget->item(i, 0) && !ui->parameters_table_widget->item(i, 0)->text().isEmpty()) {  // If not empty cell column 0
			if(!((DependencesValue*)ui->parameters_table_widget->cellWidget(i,1))->text().isEmpty()) {  // If not empty cell column 1
				parameters.insert(std::pair<std::string, std::string>(
				    		           ui->parameters_table_widget->item(i,0)->text().toStdString(),
				    		           ((DependencesValue*)ui->parameters_table_widget->cellWidget(i,1))->text().toStdString()));
			}
		}
	}

    return parameters;
}

void VisionNode::AutoSizeQTableWidget() {
    AutoSizeTableWidget<QTableWidget>();
}

void VisionNode::AutoSizeDependencesQTableWidget() {
    AutoSizeTableWidget<DependencesQTableWidget>();
}

void VisionNode::setNode(QString id, QString type, QString debug_type, std::map<std::string, std::string> dependences, std::map<std::string, std::string> parameters) {
    ui->id_line_edit->setText(id);
    ui->type_line_edit->setText(type);
    ui->debug_type_line_edit->setText(debug_type);

    for(auto dependence: dependences) {
        ui->dependences_table_widget->setItem(ui->dependences_table_widget->rowCount() - 1, 0, new QTableWidgetItem(dependence.first.data()));
        ui->dependences_table_widget->setItem(ui->dependences_table_widget->rowCount() - 2, 1, new QTableWidgetItem(dependence.second.data())); // -2 because one new line is create in time TODO remake
    }

    for(auto parameter: parameters) {
        ui->parameters_table_widget->setItem(ui->parameters_table_widget->rowCount() - 1, 0, new QTableWidgetItem(parameter.first.data()));
        ui->parameters_table_widget->setCellWidget(ui->parameters_table_widget->rowCount() - 2, 1, new DependencesValue(QString::fromStdString(parameter.second.data()))); // -2 because one new line is create in time TODO remake
    }
}
