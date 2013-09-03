#include <rqt_vision_tools/visiontree.h>

#include <rqt_vision_tools/visionnode.h>

#include <QPushButton>
#include <QStackedWidget>
#include <iostream>

VisionTree::VisionTree(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::VisionTree)
{
    ui->setupUi(this);

    connect(ui->node_list_widget, SIGNAL(currentTextChanged(QString)), this, SLOT(SetNodeWidget(QString)));
    connect(ui->add_node_push_button, SIGNAL(clicked()), this, SLOT(AddNode()));
    connect(ui->remove_node_push_button, SIGNAL(clicked()), this, SLOT(RemoveNode()));
}

VisionTree::~VisionTree()
{
    delete ui;
}


void VisionTree::SetTreePath(QString path)
{
    path_ = path;

    ui->node_list_widget->clear();
    while(ui->stacked_widget->count() > 0) {
		ui->stacked_widget->removeWidget(ui->stacked_widget->widget(0));
	}

    //parse file
    vision_tree = VisionParser::ParseVisionTree(path.toStdString());

    //add items
    for(auto node: vision_tree) {
        ui->node_list_widget->addItem(node.first.data());
        VisionNode *vision_node = new VisionNode();
        vision_node->setNode(QString::fromStdString(node.first.data()),
                             QString::fromStdString(node.second.type),
                             QString::fromStdString(node.second.debug_node),
                             node.second.dependences,
                             node.second.parameters);
        ui->stacked_widget->addWidget(vision_node);
        ui->stacked_widget->setCurrentWidget(vision_node);
    }
    ui->node_list_widget->setCurrentRow(0);
}

void VisionTree::SetNodeWidget(QString item_text) {
    for(int i = 0; i < ui->stacked_widget->count(); i++) {
        if(((VisionNode*)ui->stacked_widget->widget(i))->id() == item_text) {
            ui->stacked_widget->setCurrentIndex(i);
        }
    }
}

void VisionTree::AddNode() {
    ui->node_list_widget->addItem(ui->node_line_edit->text());
    VisionNode *vision_node = new VisionNode();
    vision_node->setNode(ui->node_line_edit->text());
    ui->stacked_widget->addWidget(vision_node);
}

void VisionTree::RemoveNode() {
    //Find current item
    QString current_node_id = ui->node_list_widget->currentItem()->text();

    //Remove node in stacked widget
	for(int ii = 0; ii < ui->stacked_widget->count(); ii++) {
		if(((VisionNode*)ui->stacked_widget->widget(ii))->id() == current_node_id) {
			ui->stacked_widget->removeWidget(ui->stacked_widget->widget(ii));
		}
	}

    //Remove node in list widget
	delete ui->node_list_widget->currentItem();
}

void VisionTree::SaveCFVT(){
	VisionParser::VisionTree vision_tree;

	for(int i = 0; i < ui->node_list_widget->count(); i++) {
		VisionParser::Node node;
		for(int ii = 0; ii < ui->stacked_widget->count(); ii++) {
			if(((VisionNode*)ui->stacked_widget->widget(ii))->id() == ui->node_list_widget->item(i)->text()) {
				node.type = ((VisionNode*)ui->stacked_widget->widget(ii))->type().toStdString();
				node.debug_node = ((VisionNode*)ui->stacked_widget->widget(ii))->debug_node().toStdString();
				node.dependences = ((VisionNode*)ui->stacked_widget->widget(ii))->dependences();
				node.parameters = ((VisionNode*)ui->stacked_widget->widget(ii))->parameters();
			}
		}
		vision_tree.insert(
		    std::pair<std::string, VisionParser::Node>(
		        ui->node_list_widget->item(i)->text().toStdString(),node));
	}
	VisionParser::SaveVisionTree(path_.toStdString(), vision_tree);
}
