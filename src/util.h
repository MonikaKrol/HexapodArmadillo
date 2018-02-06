#ifndef UTIL_H
#define UTIL_H

#include <armadillo>
#include "src/Robot/point.h"

struct rect
{
    Point3f ul; //up-left
    Point3f ur; //up-right
    Point3f dl; //down-left
    Point3f dr; //down-right
};

class joints
{
public:
    Point3f A;
    Point3f B;
    Point3f C;
    Point3f D;

    joints(Point3f a, Point3f b, Point3f c, Point3f d)
    {
        A = a;
        B = b;
        C = c;
        D = d;
    }
    joints(){};

};

#endif // UTIL_H
