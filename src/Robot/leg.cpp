#include "src/Robot/leg.h"

Leg::Leg(Point3f joint1, Point3f angles1, Point3f lengths1, Point3f signals1)
{
    legJoints = joints(joint1, joint1, joint1, joint1);
    angles = angles1;
    lengths = lengths1;
    signals = signals1;
}

void Leg::initJointPoints()
{
    calculateJointPoints();
    legEnd = legJoints.D;
    initAngles = angles;
    calculateAngles();
}

void Leg::calculateJointPoints()
{
    arma::fvec P1 = { {lengths.x, 0, 0} };
    arma::fmat R1 = arma::Mat<float>({ {cos(angles.x), 0, sin(angles.x)},
                      {0, 1, 0},
                      {-sin(angles.x), 0, cos(angles.x)} });
    arma::fvec P11 = R1*P1;
    P11 = R*P11;
    legJoints.B = Point3f(P11(0), P11(1), P11(2)) + legJoints.A;

    arma::fvec P2 = { {lengths.y, 0, 0} };
    arma::fmat R2 = arma::Mat<float>({ {cos(angles.y), -sin(angles.y), 0},
                {sin(angles.y), cos(angles.y), 0},
                {0, 0, 1} });
    arma::fvec P22 = R1*(R2*P2);
    P22 = R*P22;
    legJoints.C = Point3f(P22(0), P22(1), P22(2)) + legJoints.B;

    arma::fvec P3 = { {lengths.z, 0, 0} };
    arma::fmat R3 = arma::Mat<float>({ {cos(angles.z), -sin(angles.z), 0},
                {sin(angles.z), cos(angles.z), 0},
                { 0, 0, 1} });
    arma::fvec P33 = R1*(R3*P3);
    P33 = R*P33;
    legJoints.D = Point3f(P33(0), P33(1), P33(2)) + legJoints.C;
}

int Leg::calculateAngles()
{
    Point3f newPos = legEnd-legJoints.A; // pozycja poczatku nogi po przekszta³ceniu

    float lx = lengths.x;

    float L = sqrt(pow(newPos.x,2)+pow(newPos.z,2));
    float iksw = sqrt(pow((L-lx),2)+pow(newPos.y,2));

    float a1 = atan((L-lx)/newPos.y);
    float a2 = acos((pow(lengths.z,2)-pow(lengths.y,2)-pow(iksw,2))/((-2.)*iksw*lengths.y));
    float b = acos((pow(iksw,2)-pow(lengths.y,2)-pow(lengths.z,2))/((-2.)*lengths.y*lengths.z));

    if(newPos.x&&newPos.z)
        angles.x = -atan(newPos.z/newPos.x);
    else
        angles.x = 0;

    angles.y = -(a1+a2-0.5*arma::datum::pi);
    angles.z = (arma::datum::pi - b + angles.y);

    relAngles.x = angles.x;
    relAngles.y = angles.y;
    relAngles.z = b;

    if(angles.x != angles.x || angles.y != angles.y || angles.z!=angles.z)//nan detect
    {
        calculateJointPoints();
        return -1;
    }

    calculateJointPoints();
    calculateServoSignals();
    return 1;
}

void Leg::calculateServoSignals()
{
    float wspolczynnik = 1000/(arma::datum::pi/2 - 1.18);//wspolczynnik zamiany katow na sygnaly
    int sygnalA, sygnalB, sygnalC;
    sygnalA = relAngles.x*wspolczynnik + signals.x;
    sygnalB = -relAngles.y*wspolczynnik + signals.y;
    sygnalC = (arma::datum::pi/2 -relAngles.z)*wspolczynnik + signals.z;

    if(device)
    {
        device->setTarget(servos.x, sygnalA);
        device->setTarget(servos.y, sygnalB);
        device->setTarget(servos.z, sygnalC);
    }
}