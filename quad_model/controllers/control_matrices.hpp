#include <iostream>
#include <Eigen/Dense>
#include <cmath>

struct DroneStateMatrix {
    Eigen::MatrixXd A; // State transition matrix
    double roll, pitch, yaw; // Euler angles (phi, theta, psi)
    double force_z; // Upward force in the body frame
    double mass; // Drone mass

    // Constructor
    DroneStateMatrix(double m) : mass(m), roll(0), pitch(0), yaw(0), force_z(0) {
        A = Eigen::MatrixXd::Zero(9, 9); // 9x9 system (x, y, z, vx, vy, vz, phi, theta, psi)
        updateMatrix(); // Initialize with default values
    }

    // Function to update the A matrix based on the latest roll, pitch, and yaw
    void updateMatrix() {
        double sin_phi = std::sin(roll);
        double cos_phi = std::cos(roll);
        double sin_theta = std::sin(pitch);
        double cos_theta = std::cos(pitch);

        // Reset the matrix
        A.setZero();

        // Position derivatives
        A(0, 3) = 1; // dx/dvx
        A(1, 4) = 1; // dy/dvy
        A(2, 5) = 1; // dz/dvz

        // Velocity derivatives affected by angles
        A(3, 7) = (force_z / mass) * cos_theta;  // d(vx)/d(theta)
        A(4, 6) = -(force_z / mass) * cos_theta * cos_phi; // d(vy)/d(phi)
        A(4, 7) = (force_z / mass) * sin_theta * sin_phi;  // d(vy)/d(theta)
        A(5, 6) = (force_z / mass) * cos_theta * sin_phi;  // d(vz)/d(phi)
        A(5, 7) = -(force_z / mass) * sin_theta * cos_phi; // d(vz)/d(theta)

        // Euler angle derivatives (roll, pitch, yaw)
        A(6, 8) = 1; // d(phi)/d(psi)
        A(7, 6) = cos_phi;  // d(theta)/d(phi)
        A(7, 8) = -sin_phi; // d(theta)/d(psi)
        A(8, 6) = sin_phi / cos_theta; // d(psi)/d(phi)
        A(8, 7) = cos_phi / cos_theta; // d(psi)/d(theta)
    }

    // Function to set new angles and update the matrix
    void setAngles(double new_roll, double new_pitch, double new_yaw, double new_force_z) {
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
        force_z = new_force_z;
        updateMatrix();
    }

    // Function to print the A matrix
    void printMatrix() {
        std::cout << "A Matrix:\n" << A << "\n";
    }
};