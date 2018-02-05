#ifndef POINT_H
#define POINT_H

#include <armadillo>

class Point3f
{
private:
    float x;
    float y;
    float z;
public:
    Point3f(float x, float y, float z);
};

class Point3i
{
private:
    int x;
    int y;
    int z;
public:
    Point3i(int x, int y, int z);
};

#endif // POINT_H
