#ifndef __OSDVAR_H
#define __OSDVAR_H

#include "board.h"

#define MAX_WAYPOINTS   20


// a self contained waypoint list
typedef struct WAYPOINT_TYP {
//	float para1;
//    float para2;
//    float para3;
//    float para4;

  float x;
  float y;
  float z;

  uint16_t seq;
  uint16_t cmd;

//    uint8_t frame;
  uint8_t current;
//    uint8_t autocontinue;
} WAYPOINT, *WAYPOINT_PTR;

/* 

osd_state_struct variables should have a lifetime that flows left to right
like this:

Serial reader (Mavlink/UAVTalk/Etc) => Airlock => OSDProc

The Mavlink (mavlink_osd_state) and OSDProc (osdproc_osd_state) 
instances of this struct are owned completely by Mavlink and
OSDProc; no other threads should access them. The Airlock is
where exchange between them happens, and access to the Airlock
is controlled via a mutex. 

There are two copy operations that use
copy_osd_state_thread_safe to copy from Mavlink => Airlock, and
Airlock => OSDProc.  The idea is to have simple, ready access to
the osd_state_struct variable's contents during Mavlink reading 
and OSD writing, and to avoid ever pending on a slow serial read
operation during a fast screen drawing operation.

If that were all there were to it, it would be very clean. But some
code elsewhere (like Board.c) also updates values directly in the airlock,
and the mutex is used to control access to these as well. 

See also the other_osd_state_struct, which have a less predictable
lifetime than osd_state_struct, and have a variety of convenience 
accessors to make using them less aggravating and tedious.

-- SLG
*/
typedef struct osd_state_struct osd_state;
struct osd_state_struct {
    float osd_alt;                               // Current altitude
    
    float osd_lat;                               // GPS Latitude
    float osd_lon;                               // GPS Longitude
        
    float osd_vbat_A;                            // Battery A voltage in milivolt
    int16_t osd_curr_A;                          // Battery A current
    int8_t osd_battery_remaining_A;              // 0 to 100 <=> 0 to 1000

    float osd_pitch;                             // pitch from DCM
    float osd_roll;                              // roll from DCM
    // No usage in OSDProc yet, but it starts in Mavlink so is safe to put here.
    float osd_yaw;                               // relative heading form DCM
    float osd_heading;                           // ground course heading from GPS

    uint8_t osd_satellites_visible;              // number of satelites
    uint8_t osd_fix_type;                        // GPS lock 0-1=no fix, 2=2D, 3=3D
    double osd_hdop;                             // GPS HDOP
    
    float osd_lat2;                              // latitude GPS #2
    float osd_lon2;                              // longitude GPS #2
    uint8_t osd_satellites_visible2;             // number of satelites GPS #2
    uint8_t osd_fix_type2;                       // GPS lock 0-1=no fix, 2=2D, 3=3D GPS #2
    double osd_hdop2;

    float osd_airspeed;                          // airspeed -- NOTE, set to -1.0f by default. Was this important?
    float osd_groundspeed;                       // ground speed

    uint16_t osd_throttle;                       // throttle

    float osd_rel_alt;                           // relative altitude -- jmmods
    float osd_climb;

    float nav_roll;                              // Current desired roll in degrees
    float nav_pitch;                             // Current desired pitch in degrees
    int16_t nav_bearing;                         // Current desired heading in degrees
    
    uint16_t wp_dist;                            // Distance to active MISSION in meters
    uint8_t wp_number;                           // Current waypoint number
    float alt_error;                             // Current altitude error in meters
    float aspd_error;                            // Current airspeed error in meters/second
    float xtrack_error;                          // Current crosstrack error on x-y plane in meters

    bool motor_armed;
    bool last_motor_armed;
    uint8_t autopilot;
    uint8_t base_mode;
    uint32_t custom_mode;
    
    int16_t wp_target_bearing;                  // Bearing to current MISSION/target in degrees

    bool osd_chan_cnt_above_eight;
    uint16_t osd_chan1_raw;
    uint16_t osd_chan2_raw;
    uint16_t osd_chan3_raw;
    uint16_t osd_chan4_raw;
    uint16_t osd_chan5_raw;
    uint16_t osd_chan6_raw;
    uint16_t osd_chan7_raw;
    uint16_t osd_chan8_raw;
    uint16_t osd_chan9_raw;
    uint16_t osd_chan10_raw;
    uint16_t osd_chan11_raw;
    uint16_t osd_chan12_raw;
    uint16_t osd_chan13_raw;
    uint16_t osd_chan14_raw;
    uint16_t osd_chan15_raw;
    uint16_t osd_chan16_raw;
    uint8_t osd_rssi;                           //raw value from mavlink    
    
    float osd_windSpeed;
    float osd_windDir;

    uint8_t got_mission_counts;
    uint8_t enable_mission_count_request;
    uint16_t mission_counts;
    uint8_t enable_mission_item_request;
    uint16_t current_mission_item_req_index;
       
    uint16_t wp_counts;
    uint8_t got_all_wps;     
    WAYPOINT wp_list[MAX_WAYPOINTS];  
};

// TODO-- A Clear function to get 0's initialized as per the global initialziers
// accomplished. 


/* 
These are global variables that do NOT flow strictly from 
the serial protocols (Mavlink/Uavtalk/etc) to the OSD, but 
have other global-type lifetimes.

Not every one of these globals needs mutex protection, but
given the errors of the past it seems wise to err on the
conservative side, assuming performance problems can
be avoided. 

Please use the appropriate mutex when accessing these variables.

-- SLG
*/
typedef struct other_osd_state_struct other_osd_state;
struct other_osd_state_struct {  

    uint8_t mavbeat;
    uint32_t lastMAVBeat;
    uint32_t lastWritePanel;
    uint8_t waitingMAVBeats;
    uint8_t mav_type;
    uint8_t mav_system;
    uint8_t mav_component;
    uint8_t enable_mav_request;
    uint32_t sys_start_time;
    uint32_t heartbeat_start_time;
    uint32_t armed_start_time;
    uint32_t total_armed_time;

    float osd_total_trip_dist; 
    float osd_climb_ma[10];
    int osd_climb_ma_index; 

    float osd_curr_consumed_mah;                 // total current drawn since startup in amp-hours
    
    uint8_t osd_got_home;                        // tells if got home position or not
    float osd_home_lat;                          // home latitude
    float osd_home_lon;                          // home longitude
    float osd_home_alt;
    long osd_home_distance;                      // distance from home
    uint32_t osd_home_bearing;
    uint8_t osd_alt_cnt;                         // counter for stable osd_alt
    float osd_alt_prev;                          // previous altitude      
    
    // volatile probably isn't doing what the original author thought here.. remove now that it's mutex'd?
    volatile uint8_t current_panel;   

    float atti_mp_scale;
    float atti_3d_scale;
    uint32_t atti_3d_min_clipX;
    uint32_t atti_3d_max_clipX;
    uint32_t atti_3d_min_clipY;
    uint32_t atti_3d_max_clipY;
     
    int8_t osd_offset_X;        
    int8_t osd_offset_Y;            
};
    
/////////////////////////////////////////////////////////////////////////

// Utility functions to clear structs to zero. These require careful
// thought when used, since the various structs have very different 
// lifetimes.

void clear_osd_state_struct(osd_state * pOsd_state);
void clear_other_ost_state_struct(other_osd_state * pOther_osd_state);

void clear_certain_global_mutexed_structs();
              
/////////////////////////////////////////////////////////////////////////

void variable_mutexes_init(void);

// Airlock OSD state. Access controlled with mutex, take care!
// Values in the airlock start in Mavlink/Uavtalk/other serial protocol,
// then move to the OSD thread.
extern osd_state airlock_osd_state;

// Other OSD state. This is other globals that need mutex control,
// but may not be flowing directly out of mavlink, but rather are
// manipulated elswhere. This is more ad-hoc stuff, with less of 
// a pattern to the flow.
extern other_osd_state adhoc_osd_state;

// Copy an osd_state object in a thread-safe manner
void copy_osd_state(osd_state * p_osd_state_source, 
                    osd_state * p_osd_state_target,
                    TickType_t tick_delay);
                                
/////////////////////////////////////////////////////////////////////////
// Accessor convenience functions for accessing protected variables 
// between tasks. Hides details of mutexes and helps prevent accidental 
// misuse.
/////////////////////////////////////////////////////////////////////////

float get_atti_3d_scale();
float get_atti_mp_scale();
float get_current_consumed_mah();

uint32_t get_osd_home_bearing();
void set_osd_home_bearing(uint32_t new_osd_home_bearing);

float get_osd_home_lat();
void set_osd_home_lat(long new_osd_home_lat);

float get_osd_home_lon();
void set_osd_home_lon(long new_osd_home_lon);

long get_osd_home_distance();
void set_osd_home_distance(long new_osd_home_distance);

uint8_t get_osd_got_home();
void set_osd_got_home(uint8_t new_osd_got_home);

uint8_t get_osd_alt_cnt();
void set_osd_alt_cnt(uint8_t new_osd_alt_cnt);

uint8_t get_osd_alt_prev();
void set_osd_alt_prev(uint8_t new_osd_alt_prev);

uint8_t get_osd_home_alt();
void set_osd_home_alt(uint8_t new_osd_home_alt);

uint8_t get_current_panel();
void set_current_panel(uint8_t new_current_panel);  

uint8_t get_mavbeat();
void set_mavbeat(uint8_t new_mavbeat);

uint32_t get_lastMAVBeat();
void set_lastMAVBeat(uint32_t new_lastMAVBeat);

uint32_t get_lastWritePanel();
void set_lastWritePanel(uint32_t new_lastWritePanel);

uint8_t get_waitingMAVBeats();
void set_waitingMAVBeats(uint8_t new_waitingMAVBeats);

uint8_t get_mav_type();
void set_mav_type(uint8_t new_mav_type);

uint8_t get_mav_system();
void set_mav_system(uint8_t new_mav_system);

uint8_t get_mav_component();
void set_mav_component(uint8_t new_mav_component);

uint8_t get_enable_mav_request();
void set_enable_mav_request(uint8_t new_enable_mav_request);

uint32_t get_sys_start_time();
void set_sys_start_time(uint32_t new_sys_start_time);

uint32_t get_heartbeat_start_time();
void set_heartbeat_start_time(uint32_t new_heartbeat_start_time);

uint32_t get_armed_start_time();
void set_armed_start_time(uint32_t new_armed_start_time);

uint32_t get_total_armed_time();
void set_total_armed_time(uint32_t new_total_armed_time);


#endif
