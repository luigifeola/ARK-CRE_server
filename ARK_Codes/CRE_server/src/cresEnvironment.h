#ifndef DHTFENVIRONMENT_H
#define DHTFENVIRONMENT_H

/**
 * Author: Luigi Feola
 *
 * This is the code that specifies the specific environment used for the DHTF experiment.
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
#define SHIFTX 0 //sheffield
#define SHIFTY 1000 //sheffield
//#define SHIFTX 500 //cnr
//#define SHIFTY 500 //cnr
#define ARENA_CENTER 1000
#define ARENA_SIZE 2000
// #define KILO_DIAMETER 66 //cnr
#define KILO_DIAMETER 33 //sheffield
#define ACTIVE_AREAS 8
#define HARD_TASKS_NUMBER 4

typedef enum {
    RANDOM_WALK=0,
    INSIDE_AREA=1,
    LEAVING=2,
    PARTY=3,
}kilobot_state;

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);
    void reset();

    QVector<kilobot_state> kilobots_states; // list of all kilobots locations meaning 0 for outside areas, 1 for inside
    QVector<kilobot_state> kilobots_states_LOG;
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;  // list of all kilobots led colours, the led indicate the state of the kilobot

    QVector<Area*> areas;   // list of all areas present in the experiment
    Area* completed_area = new Area(1000, 0, 0, QPointF(1000.0,1000.0),200.0); // random values

    QVector<float> lastSent;    // when the last message was sent to the kb at given position


    int ArenaX, ArenaY;

    float minTimeBetweenTwoMsg; // minimum time between two messages
    double time;
    bool saveLOG;
    bool initialised_client = false;
    QString initialise_buffer;
    QString send_buffer;
    QString receive_buffer;
// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);


private:
    bool isTooclose(int kilobot_id);
    void initialiseEnvironment(QVector<int> activated_areas, QVector<uint> hard_tasks, QVector<uint> hard_tasks_client);
    double normAngle(double angle);
    QVector2D VectorRotation2D (double angle, QVector2D vec);
    QVector<int> proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit);
};




#endif // DHTFENVIRONMENT_H
