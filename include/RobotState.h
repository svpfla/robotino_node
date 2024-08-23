#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <urdf/model.h>
#include <string>

class RobotState
{

private:
    // position
    double x;
    double y;
    double phi;

    // velocity
    float vx;
    float vy;
    float omega;

    // sequence number to keep track of order of messages
    unsigned int sequence;

    // last motor positions
    float last_left_motor_position;
    float last_right_motor_position;

    // wheel parameters (will be read from URDF)
    float wheel_distance_;
    float wheel_radius_;

public:
    RobotState() : x(0.0), y(0.0), phi(0.0), vx(0.0), vy(0.0), omega(0.0), sequence(0), last_left_motor_position(0.0), last_right_motor_position(0.0), wheel_distance_(0.35), wheel_radius_(0.1) {}
    bool loadURDF(const std::string &urdf_file);

<<<<<<< Updated upstream
    void update(std::vector<float> motor_velocities, std::vector<int> motor_positions, double dt);
=======
    void update(std::vector<float> motor_velocities, std::vector<int> motor_positions, float dt);
>>>>>>> Stashed changes
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