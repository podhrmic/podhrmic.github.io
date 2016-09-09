#include "dialog.h"
#include "serverthread.h"

#include <QApplication>
#include <QtCore>
#include <QTextStream>


int main(int argc, char *argv[])
{
    // dialog code
    //-------------------------------------------------------
    QApplication a(argc, argv);
    Dialog w;

    w.show();

    return a.exec();
}
