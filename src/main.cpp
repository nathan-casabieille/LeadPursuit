#include "gui/MainWindow.h"

#include <QApplication>
#include <QPalette>
#include <QColor>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setStyle("Fusion");

    QPalette p;
    p.setColor(QPalette::Window,          QColor(14, 16, 26));
    p.setColor(QPalette::WindowText,      QColor(190, 205, 230));
    p.setColor(QPalette::Base,            QColor(8, 10, 18));
    p.setColor(QPalette::AlternateBase,   QColor(18, 22, 36));
    p.setColor(QPalette::ToolTipBase,     QColor(14, 16, 26));
    p.setColor(QPalette::ToolTipText,     QColor(190, 205, 230));
    p.setColor(QPalette::Text,            QColor(190, 205, 230));
    p.setColor(QPalette::Button,          QColor(20, 24, 38));
    p.setColor(QPalette::ButtonText,      QColor(190, 205, 230));
    p.setColor(QPalette::BrightText,      QColor(255, 255, 255));
    p.setColor(QPalette::Link,            QColor(90, 160, 255));
    p.setColor(QPalette::Highlight,       QColor(60, 120, 230));
    p.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    p.setColor(QPalette::Mid,             QColor(30, 38, 60));
    p.setColor(QPalette::Dark,            QColor(10, 12, 20));
    p.setColor(QPalette::Shadow,          QColor(4, 5, 10));
    app.setPalette(p);

    lead_pursuit::MainWindow window;
    window.show();

    return app.exec();
}
