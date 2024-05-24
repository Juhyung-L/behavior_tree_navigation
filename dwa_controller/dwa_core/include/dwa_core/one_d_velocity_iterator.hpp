#ifndef ONE_D_VELOCITY_ITERATOR_HPP_
#define ONE_D_VELOCITY_ITERATOR_HPP_

#include <algorithm>
#include <cmath>

namespace dwa_core
{
const double EPSILON{1E-5};

/**
 * @class OneDVelocityIterator
 * @brief A class to iterator from min achievable velocity to max achievable velocity
 * num_iteration number of times
*/

class OneDVelocityIterator
{
public:
    OneDVelocityIterator()
    {}

    /**
     * @brief Get the max and min achievable velocities within the alotted time dt given
     * current vel, acceleration, and deceleration
    */
    void initialize(double current_vel, double min_vel, double max_vel, 
        double acc, double decel, double dt, int num_samples)
    {
        if (current_vel < min_vel)
        {
            current_vel = min_vel;
        }
        else if (current_vel > max_vel)
        {
            current_vel = max_vel;
        }

        max_vel_achievable_ = projectVelocity(current_vel, acc, decel, dt, max_vel);
        min_vel_achievable_ = projectVelocity(current_vel, acc, decel, dt, min_vel);

        // number of samples need to be at least 2 because
        // 1 means you are not dividing the velocity
        // anything below 1 is invalid (can't have less than 1 sample)
        num_samples = std::max(2, num_samples);

        reset();
        increment_vel_ = std::fabs(max_vel_achievable_ - min_vel_achievable_) / (num_samples-1);
    }

    OneDVelocityIterator& operator++()
    {
        current_vel_ += increment_vel_;
        return *this;
    }

    bool isFinished()
    {
        return current_vel_ + EPSILON > max_vel_achievable_;
    }

    void reset()
    {
        current_vel_ = min_vel_achievable_;
    }

    double getCurrentVelocity()
    {
        return current_vel_;
    }

private:
    double max_vel_achievable_, min_vel_achievable_;
    double current_vel_;
    double increment_vel_;

    double projectVelocity(double current_vel, double acc, double decel, double dt, double target_vel)
    {
        if (current_vel < target_vel)
        {
            current_vel = current_vel + acc * dt;
            return std::min(current_vel, target_vel);
        }
        else
        {
            current_vel = current_vel + decel * dt;
            return std::max(current_vel, target_vel);
        }
    }
};
}

#endif