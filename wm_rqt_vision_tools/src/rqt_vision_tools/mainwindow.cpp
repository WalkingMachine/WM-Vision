#include <rqt_vision_tools/mainwindow.h>
#include <iostream>
#include <ros/ros.h>
#include <wm_vision_kernel/wm_vision_interface_flow.h>

MainWindow::MainWindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->parse_catalog_push_button, SIGNAL(clicked()), this, SLOT(LoadCatalog()));
    connect(ui->action_combo_box, SIGNAL(currentIndexChanged(QString)), this, SLOT(LoadObject(QString)));
    connect(ui->object_combo_box, SIGNAL(currentIndexChanged(QString)), this, SLOT(LoadCFVFPath(QString)));
    connect(ui->restart_push_button, SIGNAL(clicked()), this, SLOT(RestartVisionFlow()));
    connect(ui->start_push_button, SIGNAL(clicked()), this, SLOT(StartVisionFlow()));
    connect(ui->stop_push_button, SIGNAL(clicked()), this, SLOT(StopVisionFlow()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::LoadCatalog() {
    catalog = VisionParser::ParseCatalogCFVT(ui->catalog_path_line_edit->text().toStdString());

    ui->action_combo_box->clear();

    for(auto catalog_item: catalog) {
        ui->action_combo_box->addItem(catalog_item.first.data());
    }
}

void MainWindow::LoadObject(QString action) {
    ui->object_combo_box->clear();

    for(auto object: catalog[action.toStdString()]) {
        ui->object_combo_box->addItem(object.first.data());
    }
}

void MainWindow::LoadCFVFPath(QString object) {
    if (!object.isEmpty()) {
        std::string cfvf_file_name = catalog[ui->action_combo_box->currentText().toStdString()].find(object.toStdString())->second;

        ((VisionTree*)ui->vision_tree_widget)->SetTreePath(ui->vision_tree_path_line_edit->text() + QString::fromStdString(cfvf_file_name) + ".cfvf");
    }
}

void MainWindow::SaveCFVF() {
	((VisionTree*)ui->vision_tree_widget)->SaveCFVT();
}

void MainWindow::StartVisionFlow() {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<wm_vision_kernel::wm_vision_interface_flow>("wm_vision_kernel");
	wm_vision_kernel::wm_vision_interface_flow srv;
	srv.request.task_name = ui->action_combo_box->currentText().toStdString();
	srv.request.object_name = ui->object_combo_box->currentText().toStdString();
	srv.request.frequency = ui->frequency_double_spin_box->value();
	srv.request.action = "start";
	client.call(srv);
}

void MainWindow::StopVisionFlow() {
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<wm_vision_kernel::wm_vision_interface_flow>("wm_vision_kernel");
	wm_vision_kernel::wm_vision_interface_flow srv;
	srv.request.task_name = ui->action_combo_box->currentText().toStdString();
	srv.request.object_name = ui->object_combo_box->currentText().toStdString();
	srv.request.frequency = ui->frequency_double_spin_box->value();
	srv.request.action = "stop";
	client.call(srv);
}

void MainWindow::RestartVisionFlow() {
	StopVisionFlow();
	SaveCFVF();
	StartVisionFlow();
}
