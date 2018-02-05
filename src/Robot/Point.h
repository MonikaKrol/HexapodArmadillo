#ifndef POINT_H
#define POINT_H

#include <armadillo>

class Point2f
{
public:
    float x;
    float y;
    Point2f(float _x, float _y);
    Point2f();
};

class Point3f
{
public:
    float x;
    float y;
    float z;
    Point3f(float _x, float _y, float _z);
    Point3f();
    Point3f operator+(const Point3f &givenPoint3f);
    Point3f operator-(const Point3f &givenPoint3f);
    Point3f operator/(const int number);
    Point3f &operator+=(const Point3f &givenPoint3f);
    Point3f operator*(const int number);
    Point3f operator-();
};

class Point3d
{
public:
    double x;
    double y;
    double z;
    Point3d(double _x, double _y, double _z);
    Point3d();
    Point3d operator=(const Point3f &givenPoint3f);
    Point3d operator-(const Point3d &givenPoint3d);
};

class Point3i
{
public:
    int x;
    int y;
    int z;
    Point3i(int _x, int _y, int _z);
    Point3i();
};

#endif // POINT_H
