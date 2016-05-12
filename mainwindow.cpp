#include "mainwindow.h"
#include "ui_mainwindow.h"
bool MONITOR = 1;
bool storage = 0;
int slamInterval = 50;
int commInterval = 80;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    this->showFullScreen();
    ui->dockWidget->setAllowedAreas(Qt::RightDockWidgetArea);

    connect(ui->actionShowPanel, SIGNAL(triggered()), this, SLOT(on_showPanel()));
    connect(ui->actionHidePanel, SIGNAL(triggered()), this, SLOT(on_hidePanel()));

    QString library = "wheelchair";
    QString config = "/home/wheel/config/app/config.xml";

    commSourceTimer.setInterval(commInterval);
    slamTimer.setInterval(slamInterval);

    SensorInternalEvent* joystick = new SensorInternalEvent(library, "Sensor_Joystick", "joystick", config);
    joystick->setOutputNodesName(QList<QString>()<<"controljoy_viewer;control;joystick_viewer");

    VisualizationMono* joystick_viewer = new VisualizationMono(library, "Sensor_Joystick", "joystick_viewer", config);
    joystick_viewer->setInputNodesName(QList<QString>()<<"joystick");
    joystick_viewer->connectExternalTrigger(0, DRAINSLOT);

    //STM32 MCU
    SourceDrainMono* stm32comm = new SourceDrainMono(library, "Sensor_stm32comm", "stm32comm", config);
    stm32comm->setInputNodesName(QList<QString>()<<"control");
    stm32comm->setOutputNodesName(QList<QString>()<<"stm32comm_viewer;stm32comm_storage;feature;pathgenerator;combined_viewer");
    stm32comm->connectExternalTrigger(0, DRAINSLOT);
    stm32comm->connectExternalTrigger(&commSourceTimer, SIGNAL(timeout()), SOURCESLOT);

    VisualizationMono* stm32comm_viewer = new VisualizationMono(library, "Sensor_stm32comm", "stm32comm_viewer", config);
    stm32comm_viewer->setInputNodesName(QList<QString>()<<"stm32comm");
    stm32comm_viewer->connectExternalTrigger(0, DRAINSLOT);

    StorageMono* stm32comm_storage = new StorageMono(library, "Sensor_stm32comm", "stm32comm_storage", config);
    stm32comm_storage->setInputNodesName(QList<QString>()<<"stm32comm");
    stm32comm_storage->connectExternalTrigger(0, DRAINSLOT);

    SensorInternalEvent* laser = new SensorInternalEvent(library, "Sensor_Laser", "laser", config);
    laser->setOutputNodesName(QList<QString>() << "laser_viewer;laser_storage;feature");

    VisualizationMono* laser_viewer = new VisualizationMono(library, "Sensor_Laser", "laser_viewer", config);
    laser_viewer->setInputNodesName(QList<QString>() << "laser");
    laser_viewer->connectExternalTrigger(0, DRAINSLOT);

    StorageMono* laser_storage = new StorageMono(library, "Sensor_Laser", "laser_storage", config);
    laser_storage->setInputNodesName(QList<QString>()<<"laser");
    laser_storage->connectExternalTrigger(0,DRAINSLOT);

    //Slam
    SensorTimer* slam = new SensorTimer(library,"Localization_Slam", "slam", config, slamInterval);
    slam->setOutputNodesName(QList<QString>()<<"pathgenerator;feature");
    slam->connectExternalTrigger(&slamTimer, SIGNAL(timeout()), SOURCESLOT);

    //PathGenerator
    ProcessorMulti* pathgenerator = new ProcessorMulti(library, "Processor_PathGenerator", "pathgenerator", config);
    pathgenerator->setInputNodesName(QList<QString>() << "stm32comm" << "evaluate" << "slam");
    pathgenerator->setOutputNodesName(QList<QString>()<<"control;pathViewer;combined_viewer");
    pathgenerator->connectExternalTrigger(0, PROCESSORSLOT);

    VisualizationMono* pathViewer = new VisualizationMono(library, "Processor_PathGenerator", "pathViewer", config);
    pathViewer->setInputNodesName(QList<QString>() << "pathgenerator");
    pathViewer->connectExternalTrigger(0, DRAINSLOT);

    //feature extract
    ProcessorMulti* feature = new ProcessorMulti(library, "Processor_FeatureExtract", "feature", config);
    feature->setInputNodesName(QList<QString>() << "laser" << "stm32comm" << "slam");
    feature->setOutputNodesName(QList<QString>() << "evaluate;feature_viewer;combined_viewer");
    feature->connectExternalTrigger(0, PROCESSORSLOT);

    VisualizationMono* feature_viewer=new VisualizationMono(library, "Processor_FeatureExtract", "feature_viewer", config);
    feature_viewer->setInputNodesName(QList<QString>() << "feature");
    feature_viewer->connectExternalTrigger(0, DRAINSLOT);

    //evaluate
    ProcessorMono* evaluate = new ProcessorMono(library, "Processor_EvaluateFunction", "evaluate", config);
    evaluate->setInputNodesName(QList<QString>() << "feature");
    evaluate->setOutputNodesName(QList<QString>() << "pathgenerator");
    evaluate->connectExternalTrigger(0, PROCESSORSLOT);

    //Control
    ProcessorMulti* control = new ProcessorMulti(library, "Processor_Control", "control", config);
    control->setInputNodesName(QList<QString>() << "pathgenerator" << "joystick");
    control->setOutputNodesName(QList<QString>() << "controljoy_viewer;stm32comm");
    control->connectExternalTrigger(1, PROCESSORSLOT);


    //VisualMisc
    VisualizationMulti* controljoy_viewer = new VisualizationMulti(library, "VisualMisc_ControlJoy", "controljoy_viewer", config);
    controljoy_viewer->setInputNodesName(QList<QString>() << "control" << "joystick");
    controljoy_viewer->connectExternalTrigger(0, DRAINSLOT);

    VisualizationMulti* combined_viewer = new VisualizationMulti(library, "VisualMisc_CombinedVisual", "combined_viewer", config);
    combined_viewer->setInputNodesName(QList<QString>() << "feature" << "pathgenerator" << "stm32comm");
    combined_viewer->connectExternalTrigger(2, DRAINSLOT);

    edge.addNode(joystick, 1, MONITOR);
    edge.addNode(joystick_viewer, 0, 0);
    edge.addNode(stm32comm, 1, MONITOR);
    edge.addNode(stm32comm_viewer, 0, 0);
    edge.addNode(laser, 1, MONITOR);
    edge.addNode(laser_viewer, 0, 0);
    edge.addNode(feature, 1, MONITOR);
   // edge.addNode(feature_viewer, 0, 0);
//    edge.addNode(evaluate, 1, MONITOR);
//    edge.addNode(pathgenerator, 1, MONITOR);
   // edge.addNode(pathViewer, 0, 0);
    edge.addNode(control, 1, MONITOR);
    edge.addNode(controljoy_viewer, 0, 0);
//    edge.addNode(slam, 1, MONITOR);
//    edge.addNode(combined_viewer, 0, 0);


    if(storage)
    {
        edge.addNode(stm32comm_storage, 1, 0);
        edge.addNode(laser_storage, 1, 0);
    }

    edge.connectAll();

    connect(ui->open, SIGNAL(clicked()), &edge, SLOT(openAllNodesSlot()));
    connect(ui->close, SIGNAL(clicked()), &edge, SLOT(closeAllNodesSlot()));
    connect(ui->start, SIGNAL(clicked()), this, SLOT(on_start_clicked()));
    connect(ui->stop, SIGNAL(clicked()), this, SLOT(on_stop_clicked()));


    QList<QWidget *> widgets;

    widgets = combined_viewer->getVisualizationWidgets();
    ui->scrollArea->setWidget(widgets[0]);
    ui->verticalLayout_2->addWidget(widgets[1]);

    widgets = laser_viewer->getVisualizationWidgets();
    ui->scrollArea_2->setWidget(widgets[0]);

    widgets = joystick_viewer->getVisualizationWidgets();
    ui->scrollArea_3->setWidget(widgets[0]);

    widgets = stm32comm_viewer->getVisualizationWidgets();
    ui->scrollArea_4->setWidget(widgets[0]);

    widgets = controljoy_viewer->getVisualizationWidgets();
    ui->scrollArea_5->setWidget(widgets[0]);

    if(MONITOR)
        ui->tabWidget->addTab(&edge, "Monitor");
    slamIsOpen = false;
}

void MainWindow::on_start_clicked()
{
    commSourceTimer.start();
    slamTimer.start();
}

void MainWindow::on_stop_clicked()
{
    commSourceTimer.stop();
    slamTimer.stop();
    emit sig_timerstop();
}

void MainWindow::on_showPanel()
{
    ui->dockWidget->show();
}

void MainWindow::on_hidePanel()
{
    ui->dockWidget->hide();
}

MainWindow::~MainWindow()
{
    delete ui;
}
