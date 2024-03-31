#include "KinematicSolver.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <ostream>
#include <cstdlib> // Required for system()
#include <type_traits>  
#include <chrono>
#include <Eigen/Dense>
#include "KinematicPoint.h"

// Constructor implementation
KinematicSolver::KinematicSolver(double minS1, double maxS1, double minJ1, double maxJ1, double L1,
                                 double minJ2, double maxJ2, double L2, double minJ3, double maxJ3,
                                 double L3, double minS3, double maxS3, double toleranceValue,
                                 double minXValue, double minYValue, double minZValue,
                                 double maxXValue, double maxYValue, double maxZValue)
: minS1angle(minS1), maxS1angle(maxS1),
  minJ1angle(minJ1), maxJ1angle(maxJ1),
  minJ2angle(minJ2), maxJ2angle(maxJ2),
  minJ3angle(minJ3), maxJ3angle(maxJ3),
  minS3angle(minS3), maxS3angle(maxS3),
  L1length(L1), L2length(L2), L3length(L3),
  tolerance(toleranceValue),
  minX(minXValue), minY(minYValue), minZ(minZValue),
  maxX(maxXValue), maxY(maxYValue), maxZ(maxZValue) {
}

// Utility functions implementation
double KinematicSolver::degreesToRadians(double degrees) const {
    return degrees * (PI / 180.0);
}

double KinematicSolver::xPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const {
    // Example implementation, adjust as needed
    return std::cos(degreesToRadians(S1angle)) *
           (L1length * std::sin(degreesToRadians(J1angle)) +
            L2length * std::sin(degreesToRadians(J1angle + J2angle))) +
           std::cos(degreesToRadians(S1angle + S3angle)) *
           L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

double KinematicSolver::yPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const {
    // Example implementation, adjust as needed
    return std::sin(degreesToRadians(S1angle)) *
           (L1length * std::sin(degreesToRadians(J1angle)) +
            L2length * std::sin(degreesToRadians(J1angle + J2angle))) +
           std::sin(degreesToRadians(S1angle + S3angle)) * L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

double KinematicSolver::zPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) const {
    // Example implementation, adjust as needed
    return L1length * std::sin(degreesToRadians(J1angle)) +
           L2length * std::sin(degreesToRadians(J1angle + J2angle)) +
           L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

double KinematicSolver::approximatePartialDerivative(double (*func)(double, double, double, double, double), 
                                                     double S1angle, double J1angle, double J2angle, double J3angle, double S3angle, 
                                                     int angleIdx, double h) const {
    // Implementation of the partial derivative approximation
    double angles[5] = {S1angle, J1angle, J2angle, J3angle, S3angle};
    angles[angleIdx] += h;
    double forward = func(angles[0], angles[1], angles[2], angles[3], angles[4]);
    angles[angleIdx] -= 2 * h;
    double backward = func(angles[0], angles[1], angles[2], angles[3], angles[4]);
    return (forward - backward) / (2 * h);
}

/*Given an a current Kpoint (which holds 5 angles, an xyz coordinate), generate the angles needed to create the proper angles to yield the new
desired xyz coordinate. Needs a starting point, as given, as the Ax =B matrix mathematics, uses a starting point,
then begins picking the angle changes that yield the highest change towards our target position. It iterates until
it finds a point that satisfies the sensitivity.*/
KinematicPoint getKinematicPointAtoB(KinematicPoint currentkp, double desiredx, double desiredy, double desiredz)
{

    // if(desiredx < minX || desiredx > maxX ||  desiredy < minY || desiredy > maxY || desiredz < minZ || desiredz > maxZ)
    // {
    //     return =;
    // }

     double lambda = 0.0000000001; // Example damping factor, tune as needed. Higher number more stability, less precision. Lower number more precision, less stability
    //NOTICE! The lambda should be pretty small to comply with our tolerance desired. 

    double S1angle = currentkp.getAngle1();
    double J1angle = currentkp.getAngle2();
    double J2angle = currentkp.getAngle3();
    double J3angle = currentkp.getAngle4();
    double S3angle = currentkp.getAngle5();
    double currentx = currentkp.getX();
    double currenty = currentkp.getY();
    double currentz = currentkp.getZ();



    // Compute INTIIAL partial derivatives for x, y , z
    double dx_dS1 = approximatePartialDerivative(this->xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
    double dx_dJ1 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
    double dx_dJ2 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
    double dx_dJ3 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);
    double dx_dS3 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 4);

    double dy_dS1 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
    double dy_dJ1 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
    double dy_dJ2 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
    double dy_dJ3 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);
    double dy_dS3 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 4);

   // double dz_dS1 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
    double dz_dJ1 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
    double dz_dJ2 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
    double dz_dJ3 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);
    //double dz_dS3 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 4);
    // ... similarly compute other derivatives

 //// Create a 5x3 matrix
    Eigen::MatrixXd partials(3, 5);
    partials << dx_dS1, dx_dJ1, dx_dJ2, dx_dJ3, dx_dS3,
          dy_dS1, dy_dJ1, dy_dJ2, dy_dJ3, dy_dS3, 
          0, dz_dJ1, dz_dJ2, dz_dJ3, 0;

    Eigen::MatrixXd desiredxyz(3, 1);
    desiredxyz << desiredx,
                  desiredy,
                  desiredz;

    Eigen::MatrixXd currentxyz(3, 1);
    currentxyz << currentx,
                  currenty,
                  currentz;

    Eigen::MatrixXd b = desiredxyz - currentxyz;

    // Compute the SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(partials, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd delta_angles = svd.solve(b);

    while (std::abs(desiredxyz(0,0) - currentxyz(0,0)) > tolerance || std::abs(desiredxyz(1,0) - currentxyz(1,0)) > tolerance || std::abs(desiredxyz(2,0) - currentxyz(2,0)) > tolerance) 
    {
        // Update and limit each joint angle
        S1angle = std::min(std::max(S1angle + delta_angles(0, 0), minS1angle), maxS1angle);
        J1angle = std::min(std::max(J1angle + delta_angles(1, 0), minJ1angle), maxJ1angle);
        J2angle = std::min(std::max(J2angle + delta_angles(2, 0), minJ2angle), maxJ2angle);
        J3angle = std::min(std::max(J3angle + delta_angles(3, 0), minJ3angle), maxJ3angle);
        S3angle = std::min(std::max(S3angle + delta_angles(4, 0), minS3angle), maxS3angle);

        currentxyz(0, 0) = xPosition(S1angle, J1angle, J2angle, J3angle, S3angle);
        currentxyz(1,0) = yPosition(S1angle, J1angle, J2angle, J3angle, S3angle);
        currentxyz(2,0) = zPosition(S1angle, J1angle, J2angle, J3angle, S3angle);
    
        currentkp = KinematicPoint(S1angle, J1angle, J2angle, J3angle, S3angle, currentxyz(0,0), currentxyz(1,0), currentxyz(2,0));

        // Compute WORKING partial derivatives for x
        dx_dS1 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
        dx_dJ1 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
        dx_dJ2 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
        dx_dJ3 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);
        dx_dS3 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 4);

        dy_dS1 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
        dy_dJ1 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
        dy_dJ2 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
        dy_dJ3 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);
        dy_dS3 = approximatePartialDerivative(yPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 4);

    // double dz_dS1 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
        dz_dJ1 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 1);
        dz_dJ2 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 2);
        dz_dJ3 = approximatePartialDerivative(zPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 3);

        partials << dx_dS1, dx_dJ1, dx_dJ2, dx_dJ3, dx_dS3,
            dy_dS1, dy_dJ1, dy_dJ2, dy_dJ3, dy_dS3, 
            0, dz_dJ1, dz_dJ2, dz_dJ3, 0;

        

        // Update joint angles with the computed changes
        // This step depends on how your system's angles are represented
        // Example: S1angle += delta_angles(0, 0), etc.

        // Recompute the current position based on new joint angles using forward kinematics
        // currentxyz << new x, new y, new z positions based on updated angles

        // Recompute the Jacobian matrix with updated joint angles
        // Update partials matrix here

        // Compute the damped pseudo-inverse
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(partials.rows(), partials.rows());
        Eigen::MatrixXd dampedInverse = partials.transpose() * (partials * partials.transpose() + lambda * lambda * I).inverse();

        // Update b with the new current position
        b = desiredxyz - currentxyz;

        // Solve for the new delta_angles using the damped pseudo-inverse
        delta_angles = dampedInverse * b;

    }

    return currentkp;
}