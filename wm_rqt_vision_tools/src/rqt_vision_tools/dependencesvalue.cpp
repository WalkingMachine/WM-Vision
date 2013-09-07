#include <rqt_vision_tools/dependencesvalue.h>

#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>

DependencesValue::DependencesValue(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DependencesValue)
{
    ui->setupUi(this);

    connect(ui->value_line_edit, SIGNAL(textChanged(QString)), this, SLOT(setValueWidgetType(QString)));
}

DependencesValue::DependencesValue(QString value, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DependencesValue) {
    ui->setupUi(this);

    connect(ui->value_line_edit, SIGNAL(textChanged(QString)), this, SLOT(setValueWidgetType(QString)));

    setText(value);
 }

DependencesValue::~DependencesValue() {
    delete ui;
}

void DependencesValue::setText(QString value) {
    ui->value_line_edit->setText(value);
    setValueWidgetType(value);
}

QString DependencesValue::text() {
	return ui->value_line_edit->text();
}

void DependencesValue::setValueWidgetType(QString value) {
    QWidget *new_value_type_widget;
    bool is_integer, is_double;

    value.toInt(&is_integer);
    value.toDouble(&is_double);

    if(value == "true" || value == "false") {
        new_value_type_widget = new QPushButton();
        ((QPushButton*)new_value_type_widget)->setText("Toogle");
        connect((QPushButton *)new_value_type_widget, SIGNAL(clicked()), this, SLOT(ToogleBool()));
    } else if(is_integer) {
        new_value_type_widget = new QSpinBox();
        ((QSpinBox*)new_value_type_widget)->setMaximum(0x7FFFFFFF);
        ((QSpinBox*)new_value_type_widget)->setMinimum(0x80000001);
        ((QSpinBox*)new_value_type_widget)->setValue(ui->value_line_edit->text().toInt());
        connect((QSpinBox *)new_value_type_widget, SIGNAL(valueChanged(int)), this, SLOT(ChangeIntegerValue(int)));
    } else if(is_double) {
        new_value_type_widget = new QDoubleSpinBox();
        ((QDoubleSpinBox*)new_value_type_widget)->setMaximum(9999999999.99);
        ((QDoubleSpinBox*)new_value_type_widget)->setMinimum(-9999999999.99);
        ((QDoubleSpinBox*)new_value_type_widget)->setValue(ui->value_line_edit->text().toDouble());
        connect((QDoubleSpinBox *)new_value_type_widget, SIGNAL(valueChanged(double)), this, SLOT(ChangeDoubleValue(double)));
    } else {                                         // Text
        new_value_type_widget = new QWidget();
    }

		ui->horizontalLayout->removeWidget(ui->value_type_widget);
		ui->value_type_widget->close();

        ui->value_type_widget = new_value_type_widget;
        ui->horizontalLayout->addWidget(ui->value_type_widget);
        ui->horizontalLayout->update();
}

void DependencesValue::ToogleBool() {
    if (ui->value_line_edit->text() == "true")
        ui->value_line_edit->setText("false");
    else
        ui->value_line_edit->setText("true");
}

void DependencesValue::ChangeIntegerValue(int value) {
    ui->value_line_edit->setText(QString::number(value));
}

void DependencesValue::ChangeDoubleValue(double value) {
    ui->value_line_edit->setText(QString::number(value));
}
