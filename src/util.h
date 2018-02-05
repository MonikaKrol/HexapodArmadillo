#ifndef UTIL_H
#define UTIL_H

#include <armadillo>
#include "src/Robot/Point.h"
//#include <opencv2/core/core.hpp>

struct rect::Point3f
{
    Point3f ul, ur, dl, dr; //up-left up-right down-left down-right
};

struct joints::Point3f
{
    Point3f A,B,C,D;
};

#endif // UTIL_H
