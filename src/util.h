#ifndef UTIL_H
#define UTIL_H

#include <armadillo>
#include "src/Robot/Point.h"
//#include <opencv2/core/core.hpp>

struct rect
{
    Point3f ul; //up-left
    Point3f ur; //up-right
    Point3f dl; //down-left
    Point3f dr; //down-right
};

struct joints
{
    Point3f A;
    Point3f B;
    Point3f C;
    Point3f D;
};

#endif // UTIL_H
