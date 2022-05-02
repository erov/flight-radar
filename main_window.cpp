#include "main_window.h"
#include "./ui_main_window.h"

#include <iostream>

main_window::main_window(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::main_window)
{
    ui->setupUi(this);
    ui->widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->widget->set_label(ui->background_label);
    ui->widget->set_max_w(static_cast<qreal>(1920));
    ui->widget->set_max_h(static_cast<qreal>(1080));
    ui->widget->set_point_size(static_cast<qreal>(21));
    QObject::connect(ui->horizontalSlider, SIGNAL(sliderMoved(int)), ui->widget, SLOT(set_plane_number(int)));
    QObject::connect(ui->horizontalSlider_2, SIGNAL(sliderMoved(int)), ui->widget, SLOT(set_speed(int)));
}

main_window::~main_window()
{
    delete ui;
}

