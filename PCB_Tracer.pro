QT       += core gui

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

CONFIG += c++17

SOURCES += \
    customgraphicsview.cpp \
    main.cpp \
    mainwindow.cpp \
    ui.cpp

HEADERS += \
    customgraphicsview.h \
    mainwindow.h \
    ui.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
