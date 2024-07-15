#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <urdf/model.h>
#include <string>

class RobotState
{

private:
    double x;
    double y;
    double phi;

    float vx;
    float vy;
    float omega;
    unsigned int sequence;

    double angle_left;
    double angle_right;

    double wheel_distance_;
    double wheel_radius_;

public:
    RobotState() : x(0.0), y(0.0), phi(0.0), vx(0.0), vy(0.0), omega(0.0), sequence(0), angle_left(0.0), angle_right(0.0), wheel_distance_(0.35), wheel_radius_(0.1) {}
    bool loadURDF(const std::string &urdf_file);

    void update(std::vector<float> motor_velocities, std::vector<int> motor_positions);
    void setOdometry(double x, double y, double phi);

    double getX() const { return x; }
    double getY() const { return y; }
    double getPhi() const { return phi; }
    float getVx() const { return vx; }
    float getVy() const { return vy; }
    float getOmega() const { return omega; }
    unsigned int getSequence() const { return sequence; }
};

#endif // ROBOTSTATE_H