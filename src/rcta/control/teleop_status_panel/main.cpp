// system includes
#include <QtGui/QApplication>

// module includes
#include "teleopstatuspanel.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TeleopStatusPanel w;
    w.show();

    return a.exec();
}
