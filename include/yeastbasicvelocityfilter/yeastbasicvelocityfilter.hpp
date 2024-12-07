#pragma once

#include <iostream>

#include "yeastcpp/components/drive_filter.hpp"
#include "yeastbasicvelocityfilter/velocitrycommandfilter2d.hpp"

namespace yeast_motion
{
    class BasicVelocityFilter : DriveFilter
    {
        public:
            MotionCommand filter (MotionCommand command, MotionState state);
            BasicVelocityFilter(nlohmann::json config);
        
        private:
            std::unique_ptr<VelocityCommandFilter2D> filter_object;

    };
}