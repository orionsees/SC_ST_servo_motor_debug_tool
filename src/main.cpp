#include "mainwindow.h"
#include "theme.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    theme::applyDarkTheme(a);
    MainWindow w;
    w.show();
    return a.exec();
}
