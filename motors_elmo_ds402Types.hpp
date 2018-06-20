#ifndef motors_elmo_ds402_TYPES_HPP
#define motors_elmo_ds402_TYPES_HPP

#include <base/Time.hpp>

namespace motors_elmo_ds402 {
    /** How reading and writing is being triggered
     */
    struct TriggerConfiguration
    {
        enum Method
        {
            PERIODIC, SYNCHRONOUS
        };

        Method method = PERIODIC;

        // The period if method == PERIODIC
        base::Time period = base::Time::fromMilliseconds(100);
        // The number of SYNC messages between two reads if method == SYNCHRONOUS
        int syncPeriod = 1;
    };

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
