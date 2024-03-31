#include <iostream>
#include <vector>
#include <cmath>  // Include the cmath library to use sqrt
#include <ostream>
#include <cstdlib> // Required for system()
#include <type_traits>  
#include <chrono>
#include <Eigen/Dense>
#include "KinematicPoint.h"

class KinematicSolver {
public:
    // Constructor with parameters for all customizable constants
    KinematicSolver(double minS1, double maxS1, double minJ1, double maxJ1, double L1,
                    double minJ2, double maxJ2, double L2, double minJ3, double maxJ3,
                    double L3, double minS3, double maxS3, double toleranceValue,
                    double minXValue, double minYValue, double minZValue,
                    double maxXValue, double maxYValue, double maxZValue);

    KinematicPoint getKinematicPointAtoB(KinematicPoint currentkp, double desiredx, double desiredy, double desiredz);

private:
    const double minS1angle, maxS1angle;
    const double minJ1angle, maxJ1angle;
    const double minJ2angle, maxJ2angle;
    const double minJ3angle, maxJ3angle;
    const double minS3angle, maxS3angle;
    const double L1length, L2length, L3length;
    double tolerance; // Convergence tolerance
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    const double PI = 3.14159265358979323846;

    // Utility functions declarations
    double degreesToRadians(double degrees) const;
    double xPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const;
    double yPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const;
    double zPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const;
    double approximatePartialDerivative(double (*func)(double, double, double, double, double), 
                                        double S1angle, double J1angle, double J2angle, double J3angle, double S3angle, 
                                        int angleIdx, double h = 1e-5) const;
};
