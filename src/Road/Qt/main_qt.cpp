#include <QApplication>
#include "Road/Qt/RoadEditorMainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("ForceEngine Road Editor");
    app.setOrganizationName("ForceEngine");

    RoadEditorMainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
