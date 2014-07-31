#include <QtGui/QApplication>
#include "teleopstatuspanel.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TeleopStatusPanel w;
    w.show();
    
    return a.exec();
}
