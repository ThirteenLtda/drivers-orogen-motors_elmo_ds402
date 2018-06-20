#ifndef motors_elmo_ds402_TYPES_HPP
#define motors_elmo_ds402_TYPES_HPP

#include <base/Time.hpp>

namespace motors_elmo_ds402 {
    /** The fields in JointState that are read by ReadTask
     */
    struct RequiredJointStates
    {
        bool position = true;
        bool velocity = true;
        bool current_and_torque = true;
    };
}

#endif
