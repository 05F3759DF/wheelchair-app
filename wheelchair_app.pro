SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/qwt-6.1.2/include
LIBS += -L/usr/local/qwt-6.1.2/lib -lqwt

 INCLUDEPATH += /opt/ros/indigo/include

    LIBS += -L/opt/ros/indigo/lib -lroscpp
    LIBS += -L/opt/ros/indigo/lib -lrosconsole
    LIBS += -L/opt/ros/indigo/lib -lroscpp_serialization
    LIBS += -L/opt/ros/indigo/lib -lrostime
    LIBS += -L/opt/ros/indigo/lib -lxmlrpcpp
    LIBS += -L/opt/ros/indigo/lib -lcpp_common
    LIBS += -L/opt/ros/indigo/lib -lrosconsole_log4cxx
    LIBS += -L/opt/ros/indigo/lib -lrosconsole_backend_interface
    LIBS += -L/opt/ros/indigo/lib -ltf
    LIBS += -L/opt/ros/indigo/lib -ltf2
    LIBS += -L/opt/ros/indigo/lib -ltf2_ros
    LIBS += -L/opt/ros/indigo/lib -lpcl_ros_tf
    LIBS += -L/opt/ros/indigo/lib -ltf_conversions
    LIBS += -L/opt/ros/indigo/lib -lactionlib
    LIBS += -L/usr/lib/x86_64-linux-gnu -lboost_system

    LIBS += -L/usr/local/lib/ -lurg_c

 #LIBS += $$(HOME)/SDK/RobotSDK/Module/SharedLibrary/libwheelchair_Debug.so

PROJNAME = wheelchair_app
INSTTYPE = APP
include(RobotSDK_Main.pri)
