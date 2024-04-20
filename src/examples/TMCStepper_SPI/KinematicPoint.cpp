// KinematicPoint.cpp
#include "KinematicPoint.h"
#include <iostream>
#include <ostream>

// Constructor implementation
KinematicPoint::KinematicPoint(double a1, double a2, double a3, double a4, double a5, double xCoord, double yCoord, double zCoord)
    : angle1(a1), angle2(a2), angle3(a3), angle4(a4), angle5(a5), x(xCoord), y(yCoord), z(zCoord) {}

// Getters implementation
double KinematicPoint::getAngle1() const { return angle1; }
double KinematicPoint::getAngle2() const { return angle2; }
double KinematicPoint::getAngle3() const { return angle3; }
double KinematicPoint::getAngle4() const { return angle4; }
double KinematicPoint::getAngle5() const { return angle5; }
double KinematicPoint::getX() const { return x; }
double KinematicPoint::getY() const { return y; }
double KinematicPoint::getZ() const { return z; }

// Setters implementation
void KinematicPoint::setAngle1(double a1) { angle1 = a1; }
void KinematicPoint::setAngle2(double a2) { angle2 = a2; }
void KinematicPoint::setAngle3(double a3) { angle3 = a3; }
void KinematicPoint::setAngle4(double a4) { angle4 = a4; }
void KinematicPoint::setAngle5(double a5) { angle5 = a5; }
void KinematicPoint::setX(double xCoord) { x = xCoord; }
void KinematicPoint::setY(double yCoord) { y = yCoord; }
void KinematicPoint::setZ(double zCoord) { z = zCoord; }

std::ostream& operator<<(std::ostream& os, const KinematicPoint& kp) {
    return os << "Angle1: " << kp.getAngle1()
              << ", Angle2: " << kp.getAngle2()
              << ", Angle3: " << kp.getAngle3()
              << ", Angle4: " << kp.getAngle4()
              << ", Angle5: " << kp.getAngle5()
              << ", X: " << kp.getX()
              << ", Y: " << kp.getY()
              << ", Z: " << kp.getZ();
}