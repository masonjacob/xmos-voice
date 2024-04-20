// KinematicPoint.h
#ifndef KINEMATICPOINT_H
#define KINEMATICPOINT_H
#include <ostream>

class KinematicPoint {
private:
    double angle1;
    double angle2;
    double angle3;
    double angle4;
    double angle5;
    double x;
    double y;
    double z;

public:
    // Constructor
 // Default constructor
    KinematicPoint() : angle1(0), angle2(0), angle3(0), angle4(0), angle5(0), x(0), y(0), z(0){}

    KinematicPoint(double a1, double a2, double a3, double a4, double a5, double xCoord, double yCoord, double zCoord);

    // Accessors (getters)
    double getAngle1() const;
    double getAngle2() const;
    double getAngle3() const;
    double getAngle4() const;
    double getAngle5() const;
    double getX() const;
    double getY() const;
    double getZ() const;

    // Mutators (setters)
    void setAngle1(double a1);
    void setAngle2(double a2);
    void setAngle3(double a3);
    void setAngle4(double a4);
    void setAngle5(double a5);
    void setX(double xCoord);
    void setY(double yCoord);
    void setZ(double zCoord);

    // Overload the stream insertion operator declaration
    friend std::ostream& operator<<(std::ostream& os, const KinematicPoint& kp);
};

#endif // KINEMATICPOINT_H