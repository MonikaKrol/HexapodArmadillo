#ifndef POINT_H
#define POINT_H

#include <armadillo>

class Point2f
{
public:
    float x;
    float y;
    Point2f(float x, float y);
    Point2f();
};

class Point3f
{
public:
    float x;
    float y;
    float z;
    Point3f(float x, float y, float z);
    Point3f();
    Point3f operator+(const Point3f &givenPoint3f);
    Point3f operator-(const Point3f &givenPoint3f);
};

class Point3i
{
public:
    int x;
    int y;
    int z;
    Point3i(int x, int y, int z);
    Point3i();
};

#endif // POINT_H
