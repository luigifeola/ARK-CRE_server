#include "dhtfExperiment.h"
#include "dhtfEnvironment.h"

#include <QDebug>
#include <QThread>

// widgets
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QFrame>
#include <QtMath>
#include <QRadioButton>
#include <QPainter>
#include <QList>
#include <iterator>
#include <QSignalMapper>
#include <QFile>
#include <QDir>

#define STOP_AFTER 2000

// return pointer to interface!
// mykilobotexperiment can and should be completely hidden from the application
extern "C" DHTFEXPSHARED_EXPORT KilobotExperiment *createExpt()
{
    return new mykilobotexperiment();
}

/* setup the environment */
mykilobotexperiment::mykilobotexperiment() {
//     qDebug() << QString("in constructor");

    // Initialize seed
    QDateTime cd = QDateTime::currentDateTime();
    qsrand(cd.toTime_t());

    // setup the environments here
    connect(&dhtfEnvironment,SIGNAL(transmitKiloState(kilobot_message)), this, SLOT(signalKilobotExpt(kilobot_message)));
    this->serviceInterval = 100; // timestep expressed in ms


    server = new ServerStuff(this);
    connect(server, &ServerStuff::gotNewMesssage,this, &mykilobotexperiment::gotNewMesssage);
    connect(server->tcpServer, &QTcpServer::newConnection,this, &mykilobotexperiment::smbConnectedToServer);
    connect(server, &ServerStuff::smbDisconnected,this, &mykilobotexperiment::smbDisconnectedFromServer);
}



void mykilobotexperiment::on_pushButton_startServer_clicked()
{
    int port = 7001;
    if (!server->tcpServer->listen(QHostAddress::Any, port))
    {
        qDebug() << "Error!!!! The port is taken by some other service.";
        return;
    }
    connect(server->tcpServer, &QTcpServer::newConnection, server, &ServerStuff::newConnection);
    qDebug() << "Server started on port:" << port;
}

void mykilobotexperiment::on_pushButton_stopServer_clicked()
{
    if(server->tcpServer->isListening())
    {
        disconnect(server->tcpServer, &QTcpServer::newConnection, server, &ServerStuff::newConnection);

        QList<QTcpSocket *> clients = server->getClients();
        for(int i = 0; i < clients.count(); i++)
        {
            //server->sendToClient(clients.at(i), "Connection closed");
            server->sendToClient(clients.at(i), "0");
        }

        server->tcpServer->close();
        qDebug() << "Server stopped, post is closed";
    }
    else
    {
        qDebug() << "Error! Server was not running";
    }
}

void mykilobotexperiment::on_pushButton_testConn_clicked()
{
    if(server->tcpServer->isListening())
    {
        qDebug() <<
                    QString("%1 %2")
                    .arg("Server is listening, number of connected clients:")
                    .arg(QString::number(server->getClients().count()));
    }
    else
    {
        qDebug() <<
                    QString("%1 %2")
                    .arg("Server is not listening, number of connected clients:")
                    .arg(QString::number(server->getClients().count()));
    }
}

void mykilobotexperiment::smbConnectedToServer()
{
    qDebug() << "Somebody has connected";
}

void mykilobotexperiment::smbDisconnectedFromServer()
{
    qDebug() << "Somebody has disconnected";
}

void mykilobotexperiment::gotNewMesssage(QString msg)
{
    qDebug() << "Received from the client: " << msg << " time:" << this->time;
    dhtfEnvironment.receive_buffer = msg;

//    for (QTcpSocket* clientsSocket : server->getClients())
//    {
//        if (server->sendToClient(clientsSocket, QString("Reply: received [%1]").arg(msg))== -1)
//        {
//            qDebug() << "Some error occured";
//        }
//    }
}

void mykilobotexperiment::on_pushButton_send_clicked()
{
    for (QTcpSocket* clientsSocket : server->getClients())
    {
        if (server->sendToClient(clientsSocket, bufferLineEdit->text()) == -1)
        {
            qDebug() << "Some error occured";
        }
    }
}

void mykilobotexperiment::sendToClient(QString msg)
{
    qDebug() << "Sending to client: " << msg << " time:" << this->time;
    for (QTcpSocket* clientsSocket : server->getClients())
    {
        if (server->sendToClient(clientsSocket, msg) == -1)
        {
            qDebug() << "Some error occured";
        }
    }
}

/* create the GUI as a separate frame in the main ARK window */
QWidget *mykilobotexperiment::createGUI() {
//     qDebug() << QString("in create gui");

    QFrame *frame = new QFrame;
    QVBoxLayout *lay = new QVBoxLayout;
    frame->setLayout(lay);

    // add check box for saving the images
    QCheckBox *saveImages_ckb = new QCheckBox("Record experiment");
    saveImages_ckb->setChecked(true);  // start as not checked
    lay->addWidget(saveImages_ckb);
    toggleSaveImages(saveImages_ckb->isChecked());

    // add check box for logging the experiment
    QCheckBox *logExp_ckb = new QCheckBox("Log experiment");
    logExp_ckb->setChecked(true);   // start as not checked
    lay->addWidget(logExp_ckb);
    toggleLogExp(logExp_ckb->isChecked());

    QGroupBox * socketpart = new QGroupBox(tr("Sync field"));
    QFormLayout * layout2 = new QFormLayout;
    socketpart->setLayout(layout2);

    // add QPushButton to starting server
    QPushButton *start_server = new QPushButton("Start server");
    start_server->setStyleSheet("color: rgb(0, 0, 255)");
    layout2->addWidget(start_server);

    // add QPushButton to server for sending a predefined string
    QPushButton *send_some = new QPushButton("Send to client");
    send_some->setStyleSheet("color: rgb(0, 100, 0)");
    layout2->addWidget(send_some);

    // add line edit to test exchanged messages
//    QLineEdit* bufferLineEdit = new QLineEdit;
    bufferLineEdit->setPlaceholderText("buffer content");
    bufferLineEdit->setFocus();
    layout2->addWidget(bufferLineEdit);

    // add QPushButton to test server connection
    QPushButton *test_connection = new QPushButton("Test connection");
//    test_connection->setStyleSheet("color: rgb(255, 0, 0)");
    layout2->addWidget(test_connection);

    // add QPushButton to stop the server
    QPushButton *stop_server = new QPushButton("Stop server");
    stop_server->setStyleSheet("color: rgb(255, 0, 0)");
    layout2->addWidget(stop_server);

    lay->addWidget(socketpart);


    connect(start_server, SIGNAL(clicked(bool)),this, SLOT(on_pushButton_startServer_clicked()));
    connect(send_some, SIGNAL(clicked(bool)),this, SLOT(on_pushButton_send_clicked()));
    connect(test_connection, SIGNAL(clicked(bool)),this, SLOT(on_pushButton_testConn_clicked()));
    connect(stop_server, SIGNAL(clicked(bool)),this, SLOT(on_pushButton_stopServer_clicked()));

    connect(saveImages_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleSaveImages(bool)));
    connect(logExp_ckb, SIGNAL(toggled(bool)),this, SLOT(toggleLogExp(bool)));
    connect(this,SIGNAL(destroyed(QObject*)), lay, SLOT(deleteLater()));


    return frame;
}

void mykilobotexperiment::initialise(bool isResume) {
//    qDebug() << QString("in initialise");

    // generate the environments
     setupEnvironments();

    // Initialize Kilobot States:
    if (!isResume) {
        // init stuff
        emit getInitialKilobotStates();
    } else {
        // probably nothing
    }

    //POS = position, ROT = orientation, LED = color
    emit setTrackingType(POS | ROT | LED);

    QThread::currentThread()->setPriority(QThread::HighestPriority);

    savedImagesCounter = 0;
    this->time = 0;
    m_elapsed_time.start();

    // init log file operations
    // if the log checkmark is marked then save the logs
    if(logExp) {
        /********************************LOG EXPERIMENT***************************************************************************/
        // open file
        if(log_file_areas.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file_areas.close();
        }
        // log filename consist of the prefix and current date and time
        QString log_filename = log_filename_prefix + "_completedAreas_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file_areas.setFileName(log_filename);
        // open the file
        if(log_file_areas.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file_areas.fileName() << " opened";
            log_stream_areas.setDevice(&log_file_areas);
            log_stream_areas
                    << "time" << '\t'
                    << "id" << '\t'
                    << "creation" << '\t'
                    << "conclusion" << '\t'
                    <<"type" << '\t'
                    <<"kilo_on_top" <<  '\t'
                    <<"kilo_vector" << '\n';
        } else {
            qDebug() << "ERROR opening file "<< log_filename;
        }


        /********************************LOG FOR VIDEO***************************************************************************/
        // open file
        if(log_file.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file.close();
        }
        // log filename consist of the prefix and current date and time
        log_filename = log_filename_prefix + "_kilopos_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file.setFileName(log_filename);
        // open the file
        if(log_file.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file.fileName() << " opened";
            log_stream.setDevice(&log_file);
//            log_stream
//                    << "time" << '\t'
//                    << "kID" << '\t'
//                    << "colour" << '\t'
//                    << "positionX" << '\t'
//                    << "positionY" << '\t'
//                    << "orientation" << '\t'
//                    << "state" << '\n';
            //Initial state
           log_stream << this->time;
           for(int i=0; i<kilobots.size();i++){
                log_stream << "\t"
                           << kilobots[i].id << '\t'
                           << kilobots[i].colour << '\t'
                           << kilobots[i].position.x() << '\t'
                           << kilobots[i].position.y() << '\t'
                           << kilobots[i].orientation <<'\t'
                           << kilobots[i].state;
            }
            log_stream << endl;

        }
        else {
            qDebug() << "ERROR opening file "<< log_filename;
        }


        // open file
        if(log_file1.isOpen()) {
            // if it was open close and re-open again later
            // this erase the old content
            log_file1.close();
        }
        // log filename consist of the prefix and current date and time
        log_filename = log_filename_prefix + "_areapos_" + QDate::currentDate().toString("yyMMdd") + "_" + QTime::currentTime().toString("hhmmss") + ".txt";
        log_file1.setFileName(log_filename);
        // open the file
        if(log_file1.open(QIODevice::WriteOnly)) {
            qDebug() << "Log file " << log_file1.fileName() << " opened";
            log_stream1.setDevice(&log_file1);
//            log_stream1
//                    << "time" << '\t'
//                    << "id" << '\t'
//                    << "positionX" << '\t'
//                    << "positionY" << '\t'
//                    << "colour" << '\t'
//                    << "state" << '\t'
//                    << "kilobots_in_area" << '\n';
            //Initial state
            log_stream1 << this->time;
            for(Area* a : dhtfEnvironment.areas)
            {
                log_stream1 << '\t'
                            << a->id << '\t'
                            << a->position.x() << '\t'
                            << a->position.y() << '\t'
                            << (a->type == HARD_TASK ? 1:0) << '\t'       /*hard red, soft blue*/
                            << (a->completed == true ? 1:0) << '\t'
                            << a->kilobots_in_area.size();

            }
            log_stream1 << endl;
        }
        else {
            qDebug() << "ERROR opening file "<< log_filename;
        }
    }

    // if the checkbox for saving the images is checked
    if(saveImages) {
        emit saveImage(QString("./images/dhtf_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
    }

    // clear old drawings (e.g., from ID-identification)
    clearDrawings();
}

void mykilobotexperiment::stopExperiment() {

    //Close Log file
    if (log_file_areas.isOpen()){
        log_file_areas.close();
    }
    if (log_file.isOpen()){
        log_file.close();
    }
    if (log_file1.isOpen()){
        log_file1.close();
    }
}

void mykilobotexperiment::run() {
    //qDebug() << QString("in run");

    this->time=m_elapsed_time.elapsed()/1000.0; // time in seconds

    // stop after given time
    if(this->time >= STOP_AFTER) {
        // close the experiment
        this->stopExperiment();
        emit(experimentComplete());
    }


    // Update Environment
    dhtfEnvironment.time = (float)time;

    if(dhtfEnvironment.receive_buffer.startsWith("R")){
        dhtfEnvironment.initialised_client = true;
//        qDebug() << "************************************";
//        qDebug() << "*********INITIALISED ***************";
//        qDebug() << "************************************";
//        qDebug() << "************************************";
//        qDebug() << "************************************";
//        qDebug() << "************************************";
    }
    else if(dhtfEnvironment.initialised_client == false || dhtfEnvironment.receive_buffer.startsWith("M"))
    {
        sendToClient(dhtfEnvironment.initialise_buffer);
    }


    else if(dhtfEnvironment.send_buffer.startsWith("A") && qRound((this->time-last_ARK_message)*10.0f) >= ARK_message_period*10.0f )
    {
        // qDebug() << "ARK messages time: " << this->time <<" at " << QLocale("en_GB").toString( QDateTime::currentDateTime(), "hh:mm:ss.zzz");
        last_ARK_message = this->time;
        sendToClient(dhtfEnvironment.send_buffer);
        dhtfEnvironment.send_buffer.clear();
    }

    dhtfEnvironment.update();

    // BROADCAST START SIGNAL
    if(this->time <= 2.0)
    {
        kilobot_broadcast message;
        message.type = 1;
        emit broadcastMessage(message);
    }

    // update kilobots states
    emit updateKilobotStates();

    // update visualization twice per second
    if( qRound((this->time-last_env_update)*10.0f) >= env_update_period*10.0f ) {
        // clear current environment
        last_env_update = this->time;
        clearDrawings();
        clearDrawingsOnRecordedImage();

        // plot updated environment
        plotEnvironment();
    }

    // save LOG files and images for videos
    if( qRound((this->time - last_log)*10.0f) >= log_period*10.0f)
    {
        //UNCOMMENT AT RUN EXPERIMENTS
        //qDebug() << "Log time: " << this->time <<" at " << QLocale("en_GB").toString( QDateTime::currentDateTime(), "hh:mm:ss.zzz");
        //qDebug() << "LOGs saving at " << this->time*10;
        last_log = this->time;
        if(saveImages) {
            // qDebug() << "Saving Image";
            emit saveImage(QString("./images/dhtf_%1.jpg").arg(savedImagesCounter++, 5, 10, QChar('0')));
        }
        if(logExp)
        {
            log_stream << this->time;
            for(int i=0; i<kilobots.size();i++)
            {
                 log_stream << "\t"
                            << kilobots[i].id << '\t'
                            << kilobots[i].colour << '\t'
                            << kilobots[i].position.x() << '\t'
                            << kilobots[i].position.y() << '\t'
                            << kilobots[i].orientation <<'\t'
                            << kilobots[i].state;
            }
            log_stream << endl;

            log_stream1 << this->time;
            for(Area* a : dhtfEnvironment.areas)
            {
                log_stream1 << '\t'
                            << a->id << '\t'
                            << a->position.x() << '\t'
                            << a->position.y() << '\t'
                            << (a->type == HARD_TASK ? 1:0) << '\t'       /*hard red, soft blue*/
                            << (a->completed == true ? 1:0)<< '\t'
                            << a->kilobots_in_area.size();

            }
            log_stream1 << endl;

        }
    }


    // save log for areas
    if(logExp) {
        if(dhtfEnvironment.saveLOG)
        {
            dhtfEnvironment.saveLOG = false;
            qDebug() << "LOG_EXP: saving at " << this->time*10;


            log_stream_areas
                       << this->time << '\t'
                       << dhtfEnvironment.completed_area->id << '\t'
                       << dhtfEnvironment.completed_area->creation_time << '\t'
                       << dhtfEnvironment.completed_area->completed_time << '\t'
                       << int(dhtfEnvironment.completed_area->type) << '\t'
                       << dhtfEnvironment.completed_area->kilobots_in_area.size()<< '\t';

            for(int i=0; i<dhtfEnvironment.completed_area->kilobots_in_area.size(); i++)
            {
                log_stream_areas << dhtfEnvironment.completed_area->kilobots_in_area.at(i);
                if(i< dhtfEnvironment.completed_area->kilobots_in_area.size() - 1)
                {
                    log_stream_areas <<",";
                }
            }

            log_stream_areas << endl;
        }


    }
}

// Setup the Initial Kilobot Environment:
//   This is run once for each kilobot after emitting getInitialKilobotStates() signal.
//   This assigns kilobots to an environment.
void mykilobotexperiment::setupInitialKilobotState(Kilobot kilobot_entity) {
//    qDebug() << QString("in setup init kilobot state");

    // assign all kilobot to environment DHTF
    this->setCurrentKilobotEnvironment(&dhtfEnvironment);
    kilobot_id k_id = kilobot_entity.getID();

    // create a necessary list and variable for correct message timing
    if(k_id+1 > kilobots.size()) {
        kilobots.resize(k_id+1);
    }

    // this are used to solve a bug causing the app to crash after resetting
    if(dhtfEnvironment.lastSent.size() < k_id+1) {
        dhtfEnvironment.lastSent.resize(k_id+1);
    }
    if(dhtfEnvironment.kilobots_positions.size() < k_id+1) {
        dhtfEnvironment.kilobots_positions.resize(k_id+1);
    }
    if(dhtfEnvironment.kilobots_states.size() < k_id+1) {
        dhtfEnvironment.kilobots_states.resize(k_id+1);
    }
    if(dhtfEnvironment.kilobots_states_LOG.size() < k_id+1) {
        dhtfEnvironment.kilobots_states_LOG.resize(k_id+1);
    }
    if(dhtfEnvironment.kilobots_colours.size() < k_id+1) {
        dhtfEnvironment.kilobots_colours.resize(k_id+1);
    }



    dhtfEnvironment.lastSent[k_id] = dhtfEnvironment.minTimeBetweenTwoMsg;

    // TODO initialize kilobots location correctly
    dhtfEnvironment.kilobots_positions[k_id] = kilobot_entity.getPosition();
    dhtfEnvironment.kilobots_states[k_id] = RANDOM_WALK;
    dhtfEnvironment.kilobots_states_LOG[k_id] = RANDOM_WALK;
    dhtfEnvironment.kilobots_colours[k_id] = Qt::black;

    KiloLog kLog(k_id, kilobot_entity.getPosition(), 0, kilobot_entity.getLedColour());
    kLog.state=RANDOM_WALK;
    kilobots[k_id] = kLog;
    // qDebug() << "ORIENTATION IS " << kilobots[k_id].orientation;
    if(!kilobots_ids.contains(k_id))
        kilobots_ids.append(k_id);

    double timeForAMessage = 0.05; // 50 ms each message
    dhtfEnvironment.minTimeBetweenTwoMsg = kilobots_ids.size()*timeForAMessage/2.8;
    dhtfEnvironment.lastSent[k_id] = dhtfEnvironment.minTimeBetweenTwoMsg;
    // qDebug() << "Min time between two messages is" << dhtfEnvironment.minTimeBetweenTwoMsg;

}

// run once for each kilobot after emitting updateKilobotStates() signal
void mykilobotexperiment::updateKilobotState(Kilobot kilobotCopy) {
//    qDebug() << QString("in update KilobotStates");

    // update values for logging
    if(logExp) 
    {
        kilobot_id k_id = kilobotCopy.getID();
        kilobot_colour k_colour = kilobotCopy.getLedColour();
        QPointF k_position = kilobotCopy.getPosition();
        double k_rotation = qRadiansToDegrees(qAtan2(-kilobotCopy.getVelocity().y(), kilobotCopy.getVelocity().x()));
        kilobot_state k_state = dhtfEnvironment.kilobots_states[k_id];
        kilobots[k_id].updateAllValues(k_id, k_position, k_rotation, k_colour, k_state);
    }
}

// Setup Environment:
void mykilobotexperiment::setupEnvironments( ) {
//    qDebug() << QString("in setup env");
    dhtfEnvironment.reset();
}

QColor mykilobotexperiment::GetFloorColor(int track_x, int track_y) {
//    qDebug() << QString("in get floor color");

    QColor floorColour = Qt::white; // no resource
    // paint the resources
    for(Area* a : dhtfEnvironment.areas) {
        if(a->isInside(QPointF(track_x,track_y))) {
            floorColour = a->color;
            break;
        }
        if(floorColour != Qt::white)
            break;
    }

    return floorColour;
}

// Plot Environment on frame:
void mykilobotexperiment::plotEnvironment() {
    //    qDebug() << QString("In plot environment");
    // clean image
    clearDrawingsOnRecordedImage();

    // center, radius, color, thikness, text, dunno
    // drawCircle(QPointF(1000,1000), (ARENA_SIZE*SCALING/2) - 100, QColor(Qt::red), 2, "try", false);
    // drawCircle(QPointF(750,750), 735, QColor(Qt::yellow), 25, "center", true);

    // arena scaled
    std::vector<cv::Point> pos0 {Point(SHIFTX,SHIFTY), Point(SHIFTX,1000+SHIFTY)};
    drawLine(pos0,Qt::blue, 5,"",false);
    std::vector<cv::Point> pos1 {Point(SHIFTX,SHIFTY), Point(1000+SHIFTX,SHIFTY)};
    drawLine(pos1,Qt::blue, 5,"",false);
    std::vector<cv::Point> pos2 {Point(SHIFTX,1000+SHIFTY), Point(1000+SHIFTX,1000+SHIFTY)};
    drawLine(pos2,Qt::blue, 5,"",false);
    std::vector<cv::Point> pos3 {Point(1000+SHIFTX,SHIFTY), Point(1000+SHIFTX,1000+SHIFTY)};
    drawLine(pos3,Qt::blue, 5,"",false);


    // arena - 2*Kilo_diameter
    std::vector<cv::Point> bd0 {Point(SHIFTX+2*KILO_DIAMETER,SHIFTY+2*KILO_DIAMETER), Point(SHIFTX+2*KILO_DIAMETER,SHIFTY+1000-2*KILO_DIAMETER)};
    drawLine(bd0,Qt::yellow, 3,"",false);
    std::vector<cv::Point> bd1 {Point(SHIFTX+2*KILO_DIAMETER,SHIFTY+2*KILO_DIAMETER), Point(SHIFTX+1000-2*KILO_DIAMETER,SHIFTY+2*KILO_DIAMETER)};
    drawLine(bd1,Qt::yellow, 3,"",false);
    std::vector<cv::Point> bd2 {Point(SHIFTX+2*KILO_DIAMETER,SHIFTY+1000-2*KILO_DIAMETER), Point(SHIFTX+1000-2*KILO_DIAMETER,SHIFTY+1000-2*KILO_DIAMETER)};
    drawLine(bd2,Qt::yellow, 3,"",false);
    std::vector<cv::Point> bd3 {Point(SHIFTX+1000-2*KILO_DIAMETER,SHIFTY+2*KILO_DIAMETER), Point(SHIFTX+1000-2*KILO_DIAMETER,SHIFTY+1000-2*KILO_DIAMETER)};
    drawLine(bd3,Qt::yellow, 3,"",false);

     // Draw some useful position : center + 4 corners
     // drawCircle(QPoint(ARENA_CENTER,ARENA_CENTER), 30.0, QColor(Qt::black), 15, "", false);

     // drawCircle(QPoint(0,0), 30.0, QColor(Qt::yellow), 15, "", false);
     // drawCircle(QPoint(0,2.0*ARENA_CENTER), 30.0, QColor(Qt::yellow), 15, "", false);
     // drawCircle(QPoint(2.0*ARENA_CENTER,0), 30.0, QColor(Qt::yellow), 15, "", false);
     // drawCircle(QPoint(2.0*ARENA_CENTER,2.0*ARENA_CENTER), 30.0, QColor(Qt::yellow), 15, "", false);



     for(const Area* a : dhtfEnvironment.areas)
     {

         if(!a->completed)
             drawCircle(a->position, a->radius, a->color, 3, std::to_string(a->id), false);
         else
             drawCircle(a->position, a->radius, Qt::transparent, 3, std::to_string(a->id), false);



         // Draw circles on recorded images
         // if(this->saveImages)
         // {
         //     if(!a->completed)
         //         drawCircleOnRecordedImage(a->position, a->radius, a->color, 10, std::to_string(a->id));
         //     else
         //         drawCircleOnRecordedImage(a->position, a->radius, Qt::transparent, 10, std::to_string(a->id));
         // }
     }


     // Draw led on kilobots
     // for(uint k_id : this->kilobots_ids) {
     //     drawCircleOnRecordedImage(kilobots[k_id].position, 5, kilobots[k_id].colour, 5, "");
     // }
}


