QT      += charts core gui concurrent testlib

CONFIG += c++17



SOURCES += \
    GridView.cpp \
    PathAlgorithm.cpp \
    main.cpp \
    mainWindow.cpp


HEADERS += \
    GridView.h \
    PathAlgorithm.h \
    mainWindow.h


FORMS += \
    mainWindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
INCLUDEPATH += $$PWD

DISTFILES += \
    safe_append_remove_points
