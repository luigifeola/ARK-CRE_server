/**
 * Custom definition of an area
 * With the term area we refer to the single area (e.g. the single coloured circle)
 * Areas disasppears when the task on that area is accomplished
 *
 * @author Luigi Feola
 * @email luigi.feola@istc.cnr.it
 */

#ifndef AREA_H
#define AREA_H

#include <math.h>
#include <stdlib.h>

#include <QPointF>
#include <QColor>
#include <QVector>
#include <QDebug>

#define SOFT_TASK_COMPLETED 2
#define HARD_TASK_COMPLETED 6

typedef enum
{
    SOFT_TASK=0,
    HARD_TASK=1,
} Area_type;

class Area
{
public:
    uint id; // area id (0...15)
    // uint8_t other_type; /* hard or soft task on the other side */

    QPointF position; /* Center of the task */
    double radius; /* Radius of the circle to plot */
    QColor color; /* Color used to represent the area */
    bool completed;  /* Flag to understand if the task is accomplished or not */


    /* constructor */
    Area() : id(-1), position(QPointF(0,0)), radius(0), completed(true) {}

    Area(uint id, QPointF position, double radius, QColor color, bool completed) :
        id(id), position(position), radius(radius), color(color), completed(completed)
    {
    }


    /* destructor */
    ~Area(){}

    /* check if the point is inside the area */
    bool isInside(QPointF point, double threshold = 0.0) {
       return pow(point.x()-position.x(),2)+pow(point.y()-position.y(),2) <= (pow(radius-threshold,2));
    }

    /* check if the area is in range of a given point */
    bool isInRange(QPointF point, double range, double threshold = 0.0) {
       return pow(point.x()-position.x(),2)+pow(point.y()-position.y(),2) <= (pow(range-threshold,2));
    }

    void set_completed(bool comp)
    {
        this->completed = comp;
    }

    void PrintArea()
    {
        qDebug() <<
                    this->id << '\t' <<
                    this->position.x() << '\t' <<
                    this->position.y() << '\t' <<
                    (this->completed==true?"completed":"NOT completed") << '\t' <<
                    (this->color==Qt::red?"red":"blue");
    }
};

#endif // AREA_H
