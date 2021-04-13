#ifndef DHTFEXPERIMENT_H
#define DHTFEXPERIMENTH

#include "global.h"

//
#include <QObject>
#include <QFile>
//#include <QTextStream>

// these are the templates that should be used for ARK
#include "kilobot.h"
#include "kilobotexperiment.h"
#include "kilobotenvironment.h"
#include "dhtfEnvironment.h"

// there are the file for the dhtf experiment
#include "area.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Qt includes
#include <QObject>
#include <QFile>
#include <QList>
#include <QTableWidget>
#include <QSpinBox>
#include <QFormLayout>
// #include <iterator> NON INCLUSO DA DARIO??
// #include <vector> NON INCLUSO DA DARIO??
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QScrollBar>
#include <QSlider>
#include <QGroupBox>
#include <QFormLayout>
#include <QLabel>
#include <QComboBox>
#include <QFrame>
#include <QtMath>
#include <QElapsedTimer>

#include "serverStuff.h"

using namespace cv;

/**
 * @brief Log kilobot related information, Values are defined in the template file kilobot.h
 */
class KiloLog {
public:
    KiloLog() {}
    KiloLog(kilobot_id id, QPointF position, double orientation, kilobot_colour colour) :
        id(id), position(position), orientation(orientation), colour(colour) {}

    kilobot_id id;                                  // unique kilobot id
    QPointF position;                               // kilobot position on the plane
    double orientation;                             // kilobot orientation
    kilobot_colour colour;                          // kilobot led colour
    kilobot_state state;  //kilobot state (i.e. INSIDE, OUTSIDE, LEAVING)

    /** Update all log values */
    void updateAllValues(kilobot_id id, QPointF position, double orientation, kilobot_colour colour, kilobot_state state) {
        this->id = id;
        this->position = position;
        this->orientation = orientation;
        this->colour = colour;
        this->state = state;
    }

    /** Set position */
    void setPosition(QPointF position) {
        this->position = position;
    }

    /** Set orientation */
    void setOrientation(double orientation) {
        this->orientation = orientation;
    }

    void setColour(kilobot_colour colour) {
        this->colour = colour;
    }
}; /* end class kilo_log */

/**
 * @brief mykilobotexperiment is where the dhtf experiment is defined and ARK templates area extended
 * This create a separate window in the ARK GUI where one can set up experiments variables.
*/
class DHTFEXPSHARED_EXPORT mykilobotexperiment : public KilobotExperiment
{
    Q_OBJECT

public:
    mykilobotexperiment();
    virtual ~mykilobotexperiment() {delete server;}

    QWidget *createGUI();

    
    QLineEdit* bufferLineEdit = new QLineEdit;

// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void initialise(bool);
    void run();
    void stopExperiment();
//  void setupExperiment();

    void toggleSaveImages(bool toggle) { 
        saveImages = toggle; 
    }
    void toggleLogExp(bool toggle) {
        logExp = toggle;
    }

    QColor GetFloorColor(int x, int y);

    /*************************************************************************************************/
    /*****************SOCKET PART*********************************************************************/
    /*************************************************************************************************/
    void setStatus(bool newStatus);
    void receivedSomething(QString msg);
    void gotError(QAbstractSocket::SocketError err);

private slots:
    void on_pushButton_stopServer_clicked();
    void on_pushButton_startServer_clicked();
    void on_pushButton_send_clicked();
    void on_pushButton_testConn_clicked();
    void smbConnectedToServer();
    void smbDisconnectedFromServer();
    void gotNewMesssage(QString msg);

private:
    void updateKilobotState(Kilobot kilobot_entity);
    void setupInitialKilobotState(Kilobot kilobot_entity);

    //
    void setupEnvironments();
    void sendToClient(QString msg);
    void plotEnvironment();

    //
    mykilobotenvironment dhtfEnvironment;
    QTime m_elapsed_time;

    // logging variables
    bool saveImages;
    int savedImagesCounter;
    bool logExp;
    QFile log_file_areas;
    QString log_filename_prefix = "log_dhtf";
    QTextStream log_stream_areas;
    QFile log_file;
    QFile log_file1;
    QTextStream log_stream;
    QTextStream log_stream1;

    float log_period = 1.0;
    float last_log = 0.0;

    float ARK_message_period = 2.0;
    float last_ARK_message = 0.0;

    float env_update_period = 2.0;
    float last_env_update = 0.0;

    //kilo objects
    QVector < kilobot_id >  kilobots_ids;
    QVector <KiloLog> kilobots;


    ServerStuff *server;
}; /* end class mykilobotexperiment */


#endif // DHTFEXPERIMENT_H
