#pragma once

#include <iostream>
#include <cmath>
#include <limits>
#include <iomanip>

class VelocityCommandFilter2D
{
    public:

    class Velocity
    {
        public:
        float x_velocity_meters_per_second = 0.0;
        float y_velocity_meters_per_second = 0.0;

        Velocity(float x, float y)
        {
            this->x_velocity_meters_per_second = x;
            this->y_velocity_meters_per_second = y;
        }

        Velocity()
        {
            Velocity(0,0);
        }

        float norm()
        {
            return std::sqrt(std::pow(x_velocity_meters_per_second, 2) + std::pow(y_velocity_meters_per_second,2));
        }

        bool is_zero()
        {
            return std::abs(this->x_velocity_meters_per_second) <= std::numeric_limits<float>::min() &&
                   std::abs(this->y_velocity_meters_per_second) <= std::numeric_limits<float>::min();
        }

        static Velocity zero_vector()
        {
            Velocity zero_vector;
            return zero_vector;
        }

        Velocity unit_vector()
        {
            if (is_zero())
                return zero_vector();

            Velocity result
                (x_velocity_meters_per_second / norm(),
                 y_velocity_meters_per_second / norm());
            return result;
        }

        Velocity operator-(Velocity other_val)
        {
            Velocity result;
            result.x_velocity_meters_per_second = this->x_velocity_meters_per_second - other_val.x_velocity_meters_per_second;
            result.y_velocity_meters_per_second = this->y_velocity_meters_per_second - other_val.y_velocity_meters_per_second;
            return result;
        }

        Velocity operator+(Velocity other_val)
        {
            Velocity result;
            result.x_velocity_meters_per_second = this->x_velocity_meters_per_second + other_val.x_velocity_meters_per_second;
            result.y_velocity_meters_per_second = this->y_velocity_meters_per_second + other_val.y_velocity_meters_per_second;
            return result;
        }

        Velocity operator* (float value)
        {
            Velocity result;
            result.x_velocity_meters_per_second = this->x_velocity_meters_per_second * value;
            result.y_velocity_meters_per_second = this->y_velocity_meters_per_second * value;
            return result;
        }
    };

    class Configuration
    {
        public:
            float acceleration_limit_meters_per_second_per_second = 0.0;
            float deceleration_limit_meters_per_second_per_second = 0.0;
            float snap_to_zero_cutoff_velocity_meters_per_second = 0.0;
    };

    VelocityCommandFilter2D(VelocityCommandFilter2D::Configuration config, Velocity initial_condition = Velocity::zero_vector())
    {
        this->config = config;
        this->velocity = initial_condition;
    }

    Velocity reset (Velocity initial_condition)
    {
        this->velocity = initial_condition;
        return this->velocity;
    }

    Velocity step (Velocity target_velocity, float timestep_seconds)
    {
        if (target_velocity.is_zero())
        {
            if (this->velocity.norm() < this->config.snap_to_zero_cutoff_velocity_meters_per_second)
            {
                this->velocity = Velocity::zero_vector();
                return this->velocity;
            }
        }

        Velocity difference = target_velocity - velocity;
        Velocity difference_unit_vector = difference.unit_vector();
        float difference_norm = difference.norm();

        if (difference_norm <= std::numeric_limits<float>::min())
        {
            this->velocity = target_velocity;
            return this->velocity;
        }

        Velocity accel_limit_velocity = difference_unit_vector * this->config.acceleration_limit_meters_per_second_per_second * timestep_seconds;
        float accel_limit_velocity_norm = accel_limit_velocity.norm();
        Velocity decel_limit_velocity = difference_unit_vector * this->config.deceleration_limit_meters_per_second_per_second * timestep_seconds;
        float decel_limit_velocity_norm = decel_limit_velocity.norm();

        if (difference_norm < accel_limit_velocity_norm || difference_norm < decel_limit_velocity_norm) // little lazy here for now MGT
        {
            this->velocity = target_velocity;
            return this->velocity;
        }

        // Decelerating 
        if ((this->velocity + difference_unit_vector).norm() < this->velocity.norm())
        {
            this->velocity = this->velocity + decel_limit_velocity;
            return this->velocity;
        }
        else // Accelerating
        {
            this->velocity = this->velocity +  accel_limit_velocity;
            return this->velocity;
        }
    }

    private:
    VelocityCommandFilter2D() = delete;
    Velocity velocity;
    VelocityCommandFilter2D::Configuration config;
};