#include "main_window.h"

#include <QApplication>
#include <QGuiApplication>
#include <QScreen>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    main_window w;
    w.show();
    return a.exec();
}
