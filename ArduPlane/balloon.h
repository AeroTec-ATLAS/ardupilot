#pragma once

#include <AP_Param/AP_Param.h>

class BalloonRelease
{
    public:
        // constructor
        BalloonRelease(void);
        static const struct AP_Param::GroupInfo var_info[];
        AP_Int8 override_channel;
        AP_Int16 wait_time;
        AP_Int16 wait_altitude;

        void balloon_release(void);
        bool balloon_safety_check(void);
        bool pilot_release_override(void);
        void balloon_release_override(void);
        bool balloon_released;
        uint32_t balloon_release_time_ms;
        int32_t balloon_release_altitude_cm;
};
