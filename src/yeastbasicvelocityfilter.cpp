#include "yeastbasicvelocityfilter.hpp"

using namespace yeast_motion;

BasicVelocityFilter::BasicVelocityFilter(nlohmann::json json)
{
    VelocityCommandFilter2D::Configuration config;
    filter_object.reset(new VelocityCommandFilter2D(config));
}

MotionCommand BasicVelocityFilter::filter (MotionCommand command, MotionState state)
{
    VelocityCommandFilter2D::Velocity velocity;
    velocity.x_velocity_meters_per_second = command.velocity.x;
    velocity.y_velocity_meters_per_second = command.velocity.y;

    VelocityCommandFilter2D::Velocity result = filter_object->step(velocity, 0.01); // NEED TO USE THE CONFIGURATION DT HERE

    MotionCommand result_command = command;
    result_command.velocity.x = command.velocity.x;
    result_command.velocity.y = command.velocity.y;

    return result_command;
}