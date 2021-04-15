#ifndef CRESENVIRONMENT_H
#define CRESENVIRONMENT_H

/**
 * Author: Luigi Feola
 *
 * This is the code that specifies the specific environment used for the CRES experiment.
 * The environment is divided in a 4x4 grid, of theese 16 cells just ACTIVE_AREAS are activated 
 * that have to be accomplished (occupied) by the kilobots.
 * Colours for the resources are blue and red (which represent respectively a SOTF_TASK or a HARD_TASK, 
 * chainging in the quantity of required kilobots to accomplish the task).
 */
#include <QObject>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QTime>
#include <QMatrix>
#include <QList>
#include <QColor>
#include <QElapsedTimer>

#include <limits>
#include <bitset>

#include <kilobotenvironment.h>
#include "area.h"

#define SCALING 0.5
//#define SHIFTX 0 //sheffield
//#define SHIFTY 1000 //sheffield
#define SHIFTX 500 //cnr
#define SHIFTY 500 //cnr
#define ARENA_CENTER 1000
#define ARENA_SIZE 2000
#define KILO_DIAMETER 66 //cnr
//#define KILO_DIAMETER 33 //sheffield
#define CM_TO_PIXEL 20

typedef enum {
    UNCOMMITTED=0,
    COMMITTED_N=1,
    COMMITTED_S=2,
}kilobot_state;

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);
    void reset(bool offline_exp);

    QVector<kilobot_state> kilobots_states; // list of all kilobots locations meaning 0 for outside areas, 1 for inside
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;       // list of all kilobots led colours, the led indicate the state of the kilobot

    QVector<Area*> areas;                   // list of all areas present in the experiment

    QVector<float> lastSent;                // when the last message was sent to the kb at given position


    int ArenaX, ArenaY;

    // used to implement the mechanism that switch between exploration and communication (i.e., robots not moving, experiments frozen)
    double lastTransitionTime; // used to switch between exploration time and communication time
    // double lastCommunication; // used to transmit either the "communicate" or "stop communication" message three times per second
    bool isCommunicationTime; // determine if the robots are communicating or explorations

    const int exploration_time = 5;     //in seconds
    const int communication_time = 5;   //in seconds


    float minTimeBetweenTwoMsg;             // minimum time between two messages
    double time;
    bool saveLOG;
    bool initialised_client = false;
    QString initialise_buffer;              //string with init parameters
    QString send_buffer;                    //string sended to the client
    QString receive_buffer;                 //string received from the client
// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);


private:
    bool isTooclose(int kilobot_id);
    void initialiseEnvironment(QVector<uint> resource_north_id, QVector<uint> resource_south_id);
    double normAngle(double angle);
    QVector2D VectorRotation2D (double angle, QVector2D vec);
    QVector<int> proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit);
};




#endif // CRESENVIRONMENT_H
