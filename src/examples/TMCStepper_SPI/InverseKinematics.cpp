#include <iostream>
#include "KinematicPoint.h"

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <array>
#include <utility> // for std::pair
#include <algorithm>

#define EIGEN_MAX_ALIGN_BYTES 8
#define EIGEN_DONT_VECTORIZE
#define EIGEN_MAX_STATIC_ALIGN_BYTES 8
#include "./eigen-3.4.0/Eigen/Dense"

const double board_to_shoulder = 0.23150; // (meters)

const double minS1angle = -180;
const double maxS1angle = 180;

const double minJ1angle = -135;
const double maxJ1angle = 135;
const double L1length = 0.221124; //Shoulder to Elbow (meters)

const double minJ2angle = -135;
const double maxJ2angle = 135;
const double L2length = 0.223; //Elbow to wrist (meters)

const double minJ3angle = -135;
const double maxJ3angle = 135;
const double wrist_to_mount = 0.041; // (meters)
const double mount_to_end_affector = 0.095; //(meters)
const double L3length = wrist_to_mount + mount_to_end_affector; //Wrist to end affector mount (meters)

const double minS3angle = -180;
const double maxS3angle = 180;

const double maxlength = board_to_shoulder + L1length + L2length + L3length;

 // Define a tolerance for the convergence
double tolerance = 0.001; // Example value, adjust as needed

//Generate starting Kpoint for our initial position, which will be 0 for all the angles and (0,0) for the x, y positon
KinematicPoint currentkpoint = KinematicPoint(0, 0, 0, 0, 0, 0, 0, maxlength);
                                    //        S1,J1,J2,J3,S3  X, Y, Z

double maxX = L1length + L2length + L3length + board_to_shoulder;
double maxY = L1length + L2length + L3length; //X and Y dont have the additional height assistance
double maxZ = L1length + L2length + L3length;
double minX = 0;
double minY = 0;
double minZ = 0;

// Define a constant for pi.
const double PI = 3.14159265358979323846;

// Function to convert degrees to radians
double degreesToRadians(double degrees) {
    return degrees * (PI / 180.0);
}

// Function for x position
double xPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) {
    return std::cos(degreesToRadians(S1angle)) *
           (L1length * std::sin(degreesToRadians(J1angle)) + 
            L2length * std::sin(degreesToRadians(J1angle + J2angle))) + 
           std::cos(degreesToRadians(S1angle + S3angle)) *
           L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

// Function for y position
double yPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) {
    return std::sin(degreesToRadians(S1angle)) *
           (L1length * std::sin(degreesToRadians(J1angle)) + 
            L2length * std::sin(degreesToRadians(J1angle + J2angle))) +
           std::sin(degreesToRadians(S1angle + S3angle)) * L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

// Function for z position
double zPosition(double S1angle, double J1angle, double J2angle, double J3angle, double S3angle) {
    return L1length * std::sin(degreesToRadians(J1angle)) + 
           L2length * std::sin(degreesToRadians(J1angle + J2angle)) + 
           L3length * std::sin(degreesToRadians(J1angle + J2angle + J3angle));
}

// Function to approximate partial derivative
double approximatePartialDerivative(double (*func)(double, double, double, double, double), 
                                    double S1angle, double J1angle, double J2angle, double J3angle, double S3angle, 
                                    int angleIdx, 
                                    double h = 1e-5) {
    // Array to store current angles
    double angles[5] = {S1angle, J1angle, J2angle, J3angle, S3angle};

    // Modify angle at index for forward difference
    angles[angleIdx] += h;
    double forward = func(angles[0], angles[1], angles[2], angles[3], angles[4]);

    // Modify angle at index for backward difference
    angles[angleIdx] -= 2 * h;
    double backward = func(angles[0], angles[1], angles[2], angles[3], angles[4]);

    // Compute derivative
    return (forward - backward) / (2 * h);
}

// Function to calculate the magnitude of a 3D vector
double magnitudeSolver(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

// /*Given an a current Kpoint (which holds 5 angles, an xyz coordinate), generate the angles needed to create the proper angles to yield the new
// desired xyz coordinate. Needs a starting point, as given, as the Ax =B matrix mathematics, uses a starting point,
// then begins picking the angle changes that yield the highest change towards our target position. It iterates until
// it finds a point that satisfies the sensitivity.*/
KinematicPoint getKinematicPointAtoB(KinematicPoint currentkp, double desiredx, double desiredy, double desiredz)
{

    desiredz = desiredz - board_to_shoulder; //We shift down by the board to shoulder because we already have this z achieved inherintely, we only need the difference

    double magnitude = magnitudeSolver(desiredx, desiredy, desiredz);

    // if(magnitude >= maxY)
    // {
    //     return KinematicPoint(7, 7, 7, 7, 7, 7, 7, 7); //Error return
    // }

    std::cout << "Starting" << std::endl;



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
    double dx_dS1 = approximatePartialDerivative(xPosition, S1angle, J1angle, J2angle, J3angle, S3angle, 0);
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

        std::cout << "Looping" << std::endl;

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