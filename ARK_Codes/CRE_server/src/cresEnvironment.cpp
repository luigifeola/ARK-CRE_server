#ifndef CRESENVIRONMENT_CPP
#define CRESENVIRONMENT_CPP

#include "cresEnvironment.h"
#include "area.h"

#include "kilobot.h"

#include <math.h>
#include <stdlib.h>
#include <random>

#include <QVector>
#include <QVector2D>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>

namespace {
    const QVector2D up_direction (0.0,-1.0);
    const QVector2D down_direction (0.0,1.0);
    const QVector2D left_direction (1.0,0.0);
    const QVector2D right_direction (-1.0,0.0);
    const double reachable_distance = (ARENA_SIZE*SCALING/2) - (3.0/2.0*KILO_DIAMETER);
    const int num_sectors = 6;

    const int kTotalResourceNum = 100;
    const int kNorthResourceNum = 30;
    const int kSouthResourceNum = 15;
    const double kVisionRange = 8.0 * CM_TO_PIXEL;
}

double mykilobotenvironment::normAngle(double angle){
    while (angle > 180) angle = angle - 360;
    while (angle < -180) angle = angle + 360;
    return angle;
}

QVector2D mykilobotenvironment::VectorRotation2D (double angle, QVector2D vec){
    // qDebug() << "2D Rotation";
    QVector2D rotated_vector;
    double kx = (cos(angle)* vec.x()) + (-1.0*sin(angle) * vec.y());
    double ky = (sin(angle) * vec.x()) + (cos(angle) * vec.y());
    rotated_vector.setX(kx);
    rotated_vector.setY(ky);
    return rotated_vector;
}

QVector<int> mykilobotenvironment::proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit){
    double sector = M_PI_2 / (num_bit/2.0);
    QVector<int> proximity;
    // qDebug() << "kilo_ori" << qRadiansToDegrees(kilo_rotation);
    for(int i=0; i<num_bit; i++)
    {
        QVector2D sector_dir_a = VectorRotation2D((kilo_rotation+M_PI_2 - i * sector), left_direction);
        QVector2D sector_dir_b = VectorRotation2D((kilo_rotation+M_PI_2 - (i+1) * sector), left_direction);

        // qDebug() << "wall-dir" << obstacle_direction;
        // qDebug() << "a-dir" << sector_dir_a;
        // qDebug() << "b-dir" << sector_dir_b;

        if( QVector2D::dotProduct(obstacle_direction, sector_dir_a) >=0 ||
            QVector2D::dotProduct(obstacle_direction, sector_dir_b) >=0    )
        {
            proximity.push_back(0);
        }
        else{
            proximity.push_back(1);
        }
    }

    return proximity;
}

mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    // environment specifications
    this->ArenaX = 0.45;
    this->ArenaY = 0.45;

    this->saveLOG = false;
    this->send_buffer = "";
    this->receive_buffer = "";

    this->initialise_buffer = "";
    // define environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    // reset(true);
}

void mykilobotenvironment::initialiseEnvironment(QVector<uint> resource_north_id, QVector<uint> resource_south_id){

    double white_space = SCALING * 2 * KILO_DIAMETER;
    double radius = (((2.0*ARENA_CENTER*SCALING - white_space)/ 10.0) - white_space)/2.0;
    // qDebug() << QString("radius") << radius;
    QPointF areasOffset(SHIFTX,SHIFTY);

    for (int areaID=0; areaID<kTotalResourceNum; ++areaID){
        QPointF areaPos((1.0+2.0*(areaID%10))*radius + (1.0+(areaID%10))*white_space, (1.0 + floor(areaID/10.0)*2.0 )*radius + (1.0 + floor(areaID/10.0))*white_space);
        areaPos += areasOffset;
        if(areaID<(kTotalResourceNum/2))
        {
            if(std::find(resource_north_id.begin(),resource_north_id.end(), areaID) != resource_north_id.end())
                areas.push_back(new Area(areaID, areaPos, radius, Qt::red, false));
            else
                areas.push_back(new Area(areaID, areaPos, radius, Qt::red, true));
        }
        else
        {
            if(std::find(resource_south_id.begin(),resource_south_id.end(), areaID) != resource_south_id.end())
                areas.push_back(new Area(areaID, areaPos, radius, Qt::blue, false));
            else
                areas.push_back(new Area(areaID, areaPos, radius, Qt::blue, true));
        }
    }

}

void mykilobotenvironment::reset(bool offline_exp){
    this->time = 0;
    this->minTimeBetweenTwoMsg = 0;

    areas.clear();
    kilobots_states.clear();

    kilobots_positions.clear();
    kilobots_colours.clear();

    std::default_random_engine re;
//    re.seed(0);
    re.seed(qrand());

    if(offline_exp)
    {
        QVector<uint> resource_north_id;
        QVector<uint> resource_south_id;

        int start = 0;
        int end = (kTotalResourceNum/2)-1;

        while(resource_north_id.size()<kNorthResourceNum)
        {
            std::uniform_int_distribution<uint> distr(start, end);
            uint random_number;
            do{
                random_number = distr(re);
            // qDebug() << QString("Drawing the number: ") << random_number;
            }while (std::find(resource_north_id.begin(),resource_north_id.end(), random_number) != resource_north_id.end());
            resource_north_id.push_back(random_number);
        }
        std::sort(resource_north_id.begin(), resource_north_id.end());

        // Show selected hard tasts for server
        qDebug() << QString("Selected north resources");
        qDebug() << resource_north_id;


        while(resource_south_id.size()<kSouthResourceNum)
        {
            std::uniform_int_distribution<uint> distr(start, end);
            uint random_number;
            do{
                random_number = distr(re);
            // qDebug() << QString("Drawing the number: ") << random_number;
            }while (std::find(resource_south_id.begin(),resource_south_id.end(), random_number) != resource_south_id.end());
            resource_south_id.push_back(random_number+(kTotalResourceNum/2));
        }
        std::sort(resource_south_id.begin(), resource_south_id.end());


        // Show selected hard tasts for client
        qDebug() << QString("Selected south resources");
        qDebug() << resource_south_id;

        initialiseEnvironment(resource_north_id, resource_south_id);
    }

    else{
        qDebug() << "******WAITING FOR INITIALISATION";
    }


//    // preparint initialise ("I") server message
//    QVector<int> server_task (activated_areas.size(), 0);
//    QVector<int> client_task (activated_areas.size(), 0);

//    initialise_buffer = "I";

//    for(int i=0; i<activated_areas.size(); i++)
//    {
//        int char_id = 97+activated_areas[i];    // 97 is a in ASCII table
//        initialise_buffer.append(QChar(char_id));

//        if(std::find(hard_tasks.begin(),hard_tasks.end(), activated_areas[i]) != hard_tasks.end())
//            server_task[i] = 1;

//        if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), activated_areas[i]) != hard_tasks_client.end())
//            client_task[i] = 1;

//    }

//    qDebug() << "server " << server_task;
//    qDebug() << "client " << client_task;

//    for(uint s_task : server_task)
//    {
//        initialise_buffer.append(QString::number(s_task));
//    }
//    for(uint c_task : client_task)
//    {
//        initialise_buffer.append(QString::number(c_task));
//    }




}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {
    //eventualmente sarà qui che gestirai il completamento delle aree
    send_buffer = "A";
    for(int i = 0; i < areas.size(); i++)
    {
        if(areas[i]->completed)
            send_buffer.append(QString::number(1));
        else
            send_buffer.append(QString::number(0));
    }
}



// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {   
    // qDebug() << QString("In updateVirtualSensor");
    // update local arrays
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();

    // qDebug() << QString("saving colours");
    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED){
        this->kilobots_colours[k_id] = Qt::red;
        this->kilobots_states[k_id] = COMMITTED_N;
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;
        this->kilobots_states[k_id] = COMMITTED_S;
    }
    else
    {
        this->kilobots_colours[k_id] = Qt::black;
        this->kilobots_states[k_id] = UNCOMMITTED;
    }


    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg){
        /* Prepare the inividual kilobot's message                   */
        /* see README.md to understand about ARK messaging           */
        /* data has 3x24 bits divided as                             */
        /*   ID 10b    type 4b  data 10b     <- ARK msg              */
        /*  data[0]   data[1]   data[2]      <- kb msg               */
        /* xxxx xxyy yyyy zzzz zzww wwww     <- cres                 */
        /* x bits used for kilobot id                                */
        /* y bits used for north resources                           */
        /* z bits used for south resources                           */
        /* w bits used for wall avoidance                            */

        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.id = 0;
        message.type = 0;
        message.data = 0;

        /**********************EXPERIMENT STUFF****************************/
        double total_north_area = 0.0, north_area_on = 0.0, total_south_area = 0.0, south_area_on = 0.0;
        for(int id=0; id<areas.size(); id++)
        {
            if(areas[id]->isInRange(kilobot_entity.getPosition(), kVisionRange))
            {
                if(areas[id]->color == Qt::red)
                {
                    total_north_area++;
                    if(areas[id]->completed == false)
                    {
                        north_area_on ++;
                    }
                }

                if(areas[id]->color == Qt::blue)
                {
                    total_south_area++;
                    if(areas[id]->completed == false)
                    {
                        south_area_on ++;
                    }
                }
            }
        }

        /***********************WALL AVOIDANCE STUFF***********************/
        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall
        uint8_t proximity_decimal = 0;  // 0 no turn, 1 pi/2, 2 pi, 3 3pi/2

        QPoint k_center ((ARENA_CENTER*SCALING)+SHIFTX, (ARENA_CENTER*SCALING)+SHIFTY);
        // get position translated w.r.t. center of arena
        QVector2D k_pos = QVector2D(this->kilobots_positions[k_id]);
        QVector2D shifted_pos((k_pos.x() - k_center.x()), -1.0*(k_pos.y() - k_center.y()) );
        // qDebug() << "pos:" << k_pos.x() << ' ' << k_pos.y() << "\tShifted:" << shifted_pos.x() << ' ' <<  shifted_pos.y();



        if( abs(shifted_pos.x()) > reachable_distance ||
                 abs(shifted_pos.y()) > reachable_distance )
        {
            // qDebug() << " COLLISION for kilobot " << k_id << " in position "<< kilobots_positions[k_id].x() << " " << kilobots_positions[k_id].y()
            //                                                             << " orientation " << qAtan2(QVector2D(kilobot_entity.getVelocity()).y(), QVector2D(kilobot_entity.getVelocity()).x());
            // get position translated w.r.t. center of arena


            // get orientation (from velocity)
            QVector2D k_ori = QVector2D(kilobot_entity.getVelocity());
            k_ori.setX(k_ori.x()*10);
            k_ori.setY(k_ori.y()*10);
            // qDebug() << "Orientation: " << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) );

            double k_rotation = qAtan2(-k_ori.y(), k_ori.x());
            // double angle = k_rotation - qAtan2(shifted_pos.y(), shifted_pos.x());
            // get angle wrt center position
            /*qDebug() << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) )
                     << normAngle( qRadiansToDegrees(qAtan2(shifted_pos.y(), shifted_pos.x())) ) ;
                     << normAngle( qRadiansToDegrees(angle) );*/

            // qDebug() << "Pos:" << k_pos << "\tRotation" << k_rotation;
            QVector<int> proximity;
            // TODO : Understand here what you need
            if(shifted_pos.x() > reachable_distance){
                // qDebug()<< "RIGHT";
                proximity = proximity_sensor(right_direction, k_rotation, num_sectors);
            }
            else if(shifted_pos.x() < -1.0*reachable_distance){
                // qDebug()<< "LEFT";
                proximity = proximity_sensor(left_direction, k_rotation, num_sectors);
            }

            if(shifted_pos.y() > reachable_distance){
                // qDebug()<< "UP";
                if(proximity.isEmpty())
                    proximity = proximity_sensor(up_direction, k_rotation, num_sectors);
                else{
                    QVector<int> prox = proximity_sensor(up_direction, k_rotation, num_sectors);
                    QVector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform( proximity.begin(), proximity.end(), prox.begin(),
                            std::back_inserter( elementwiseOr ), std::logical_or<int>() );

                    proximity = elementwiseOr;
                }
            }
            else if(shifted_pos.y() < -1.0*reachable_distance){
                // qDebug()<< "DOWN";
                if(proximity.isEmpty())
                    proximity = proximity_sensor(down_direction, k_rotation, num_sectors);
                else{
                    QVector<int> prox = proximity_sensor(down_direction, k_rotation, num_sectors);
                    QVector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform( proximity.begin(), proximity.end(), prox.begin(),
                            std::back_inserter( elementwiseOr ), std::logical_or<int>() );

                    proximity = elementwiseOr;
                }
            }

            proximity_decimal = std::accumulate(proximity.begin(), proximity.end(), 0, [](int x, int y) { return (x << 1) + y; });
            // qDebug() <<proximity << "\tDec: " << proximity_decimal;

            /**To turn off wall avoidance decomment the following line*/
            //proximity_decimal = 0;
        }

        message.id = k_id<<4;
        message.type = 0;
        message.data = proximity_decimal;

        qDebug() << "time:"<<this->time
                 << " MESSAGE to " << k_id
                 << "type " << message.type
                 << "payload " << message.data;
        lastSent[k_id] = this->time;
        emit transmitKiloState(message);


    }

#endif // CRESENVIRONMENT_CPP


}




