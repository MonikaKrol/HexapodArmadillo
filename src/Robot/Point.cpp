#include <armadillo>
#include "Point.h"

Point2f::Point2f(float x, float y)
{
    Point2f = { {float x, float y} };
}

Point3f::Point3f(float x, float y, float z)
{
    Point3f = { {float x, float y, float z} };
}

Point3f Point3f::operator+(const Point3f &givenPoint3f) {
    return Point3f();
}

Point3f Point3f::operator-(const Point3f &givenPoint3f) {
    return Point3f();
}

Point3i::Point3i(int x, int y, int z)
{
    Point3i = { {int x, int y, int z} };
}