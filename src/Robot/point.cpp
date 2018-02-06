#include <armadillo>
#include "point.h"

Point2f::Point2f(float _x, float _y) {
    x = _x;
    y = _y;
}

Point2f::Point2f() {
    x = 0;
    y = 0;
}

Point3f::Point3f(float _x, float _y, float _z) {
    x = _z;
    y = _y;
    z = _z;
}

Point3f::Point3f() {
    x = 0;
    y = 0;
    z = 0;
}

Point3f Point3f::operator+(const Point3f &givenPoint3f) {
    Point3f temp;
    temp.x = x + givenPoint3f.x;
    temp.y = y + givenPoint3f.y;
    temp.z = z + givenPoint3f.z;

    return temp;
}

Point3f Point3f::operator-(const Point3f &givenPoint3f) {
    Point3f temp;

    temp.x = x - givenPoint3f.x;
    temp.y = y - givenPoint3f.y;
    temp.z = z - givenPoint3f.z;

    return temp;
}

Point3f Point3f::operator/(const int number) {
    Point3f temp;

    temp.x = x / number;
    temp.y = y / number;
    temp.z = z / number;

    return temp;
}

Point3f &Point3f::operator+=(const Point3f &givenPoint3f) {
    this->x += givenPoint3f.x;
    this->y += givenPoint3f.y;
    this->z += givenPoint3f.z;

    return *this;
}

Point3f Point3f::operator*(const int number) {
    Point3f temp;

    temp.x = x * number;
    temp.y = y * number;
    temp.z = z * number;

    return temp;
}

Point3f Point3f::operator-() {
    Point3f temp;

    temp.x = -x;
    temp.y = -y;
    temp.z = -z;

    return temp;
}

Point3d::Point3d(double _x, double _y, double _z) {
    x = _z;
    y = _y;
    z = _z;
}

Point3d::Point3d() {
    x = 0;
    y = 0;
    z = 0;
}

Point3d Point3d::operator=(const Point3f &givenPoint3f) {
    Point3d temp;

    temp.x = givenPoint3f.x;
    temp.y = givenPoint3f.y;
    temp.z = givenPoint3f.z;

    return temp;
}

Point3d Point3d::operator-(const Point3d &givenPoint3d) {
    Point3d temp;

    temp.x = x - givenPoint3d.x;
    temp.y = y - givenPoint3d.y;
    temp.z = z - givenPoint3d.z;

    return temp;
}

Point3i::Point3i(int _x, int _y, int _z) {
    x = _z;
    y = _y;
    z = _z;
}

Point3i::Point3i() {
    x = 0;
    y = 0;
    z = 0;
}