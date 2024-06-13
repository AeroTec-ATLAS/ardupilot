#include "Plane.h"


/* 
   trigger release servo for balloon release
*/
void Plane::balloon_release()
{
    // If aircraft is armed, set released flag to stop servo closing
    if (!plane.balloon_released && arming.is_armed()) {
        plane.balloon_released = true;
    }
    // Activate release servo
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, 1.0f);
    // Wait 3 seconds after release to complete command
    //uint32_t time_release_ms = millis();
    // while (millis() - time_release_ms < 3000) {
    //     // Wait for 3 seconds after release
    // }
}

void Plane::balloon_release_override()
{
    // If aircraft is armed, set released flag to stop servo closing
    if (!plane.balloon_released && arming.is_armed()) {
        plane.balloon_released = true;
        gcs().send_text(MAV_SEVERITY_INFO,"Manual balloon release activated");
    }
    // Activate release servo
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, 1.0f);
}

bool Plane::pilot_release_override(){
    if (plane.balloon_released){
        plane.balloon_release_override();
        return true;
    }
    // Get position input from Scripting8 RC channel
    int switchPWM = rc().channel((uint8_t) 6)->get_radio_in();
    // Check if switch is in the release position (above 0.5)
    if (switchPWM > 1500) {
        // Activate release servo
        plane.balloon_release_override();
        return true;
    }
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting16, -1.0f);
    return false;
}

bool Plane::balloon_safety_check()
{
    // If RC failsafe is active, return true
    if (failsafe.rc_failsafe) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"EMERGENCY BALLOON RELEASE: RC failsafe activated");
        plane.balloon_release();
        // Trigger short failsafe action
        failsafe_short_on_event(FAILSAFE_SHORT, ModeReason::RADIO_FAILSAFE);
        return true;
    }
  return false;
}

