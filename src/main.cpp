#include "gui/MainWindow.h"

#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    lead_pursuit::MainWindow window;
    window.show();

    return app.exec();
}
