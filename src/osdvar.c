/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/*
 * With Grateful Acknowledgements to the projects:
 * MinimOSD - arducam-osd Controller(https://code.google.com/p/arducam-osd/)
 */
#include "osdvar.h"

/////////////////////////////////////////////////////////////////////////

// This is the OSD state that is unowned by any particular thread, and flows from
// Mavlink/UAVTalk to the OSD thread. This is called the "airlock".
osd_state airlock_osd_state = {};

// This is other global OSD state that doesn't flow from a serial protocol to OSD,
// but follows other patterns.
other_osd_state adhoc_osd_state = {};

// Clears out entire struct
void clear_osd_state_struct(osd_state * pOsd_state) {        
    // Set all bytes to 0
    memset(pOsd_state, 0, sizeof(osd_state));
    // Set legal defaults
    
}

// Clears out entire struct
void clear_other_ost_state_struct(other_osd_state * pOther_osd_state) {
    // Set all bytes to 0
    memset(pOther_osd_state, 0, sizeof(other_osd_state));
    // Default current panel to 1
    pOther_osd_state->current_panel = 1;
}

/////////////////////////////////////////////////////////////////////////
// Mutexes to protect safe access to variables shared 
// between concurrent tasks.
/////////////////////////////////////////////////////////////////////////

// This mutex controls access to the Mavlink OSD State
extern xSemaphoreHandle osd_state_mavlink_mutex;

// This mutex controls access to the airlock OSD State
xSemaphoreHandle osd_state_airlock_mutex;

// This mutex controls access to the OSDProc OSD State
extern xSemaphoreHandle osd_state_osdproc_mutex;

// This mutex controls access to the ad-hoc OSD State
xSemaphoreHandle osd_state_adhoc_mutex;

// Clear certain global mutex'd structs to zeros
// For use during startup, but not after. (The airlock
// values persist between copy operations, and there
// some values that begin their lives in the airlock.)
void clear_certain_global_mutexed_structs() {
    // The airlock has no specific owner, so we get this cleared here,
    // rather than in a specific thread like osdproc or osdmavlink.
    clear_osd_state_struct(&airlock_osd_state);
    // The ad-hoc state has a global lifetime like the airlock, so again
    // we clear it only here, once, at startup.
    clear_other_ost_state_struct(&adhoc_osd_state);
}

// Initialize the various mutexes designed to protect variables shared between tasks.
void variable_mutexes_init() {

    osd_state_mavlink_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_mavlink_mutex);

    osd_state_airlock_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_airlock_mutex);
    
    osd_state_adhoc_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_adhoc_mutex);

    osd_state_osdproc_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(osd_state_osdproc_mutex);
}

// Copy an osd_state object in a thread-safe manner
// May not succeed if the lock is not gained in tick_delay ticks
void copy_osd_state(osd_state * p_osd_state_source, 
                    osd_state * p_osd_state_target,
                    TickType_t tick_delay) {    
    if (xSemaphoreTake(osd_state_airlock_mutex, tick_delay) == pdTRUE ) {
        // Note that we deliberately DO NOT call clear_osd_state_struct() on the target -
        // this function can be used to copy TO the airlock, and there are values that
        // start their lifetime IN the airlock, and doing so would erase them.
        
        // Copy current values
        *p_osd_state_target = *p_osd_state_source;
        // Release the airlock mutex
        xSemaphoreGive(osd_state_airlock_mutex);
    }    
    else
    {
        // Did not succeed; values won't be copied.
    }
}

/////////////////////////////////////////////////////////////
/// Convenience accessors
/////////////////////////////////////////////////////////////

float get_atti_3d_scale() {
  float atti_3d_scale = 0.0f;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      atti_3d_scale = adhoc_osd_state.atti_3d_scale;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return atti_3d_scale;
}

float get_atti_mp_scale() {
  float atti_mp_scale = 0.0f;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      atti_mp_scale = adhoc_osd_state.atti_mp_scale;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return atti_mp_scale;
}

float get_current_consumed_mah() {
  float current_consumed_mah = 0.0f;  
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      current_consumed_mah = adhoc_osd_state.osd_curr_consumed_mah;
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
   return current_consumed_mah;
}

uint32_t get_osd_home_bearing() {
  uint32_t TEMP_osd_home_bearing = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_home_bearing =  adhoc_osd_state.osd_home_bearing;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_home_bearing;
}

void set_osd_home_bearing(uint32_t new_osd_home_bearing) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_home_bearing = new_osd_home_bearing;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

float get_osd_home_lat() {
  float TEMP_osd_home_lat = 0.0f;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_home_lat =  adhoc_osd_state.osd_home_lat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_home_lat;
}

void set_osd_home_lat(long new_osd_home_lat) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_home_lat = new_osd_home_lat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

float get_osd_home_lon() {
  float TEMP_osd_home_lon = 0.0f;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_home_lon = adhoc_osd_state.osd_home_lon;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_home_lon;
}

void set_osd_home_lon(long new_osd_home_lon) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_home_lon = new_osd_home_lon;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

long get_osd_home_distance() {
  long TEMP_osd_home_distance = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_home_distance = adhoc_osd_state.osd_home_distance;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_home_distance;
}

void set_osd_home_distance(long new_osd_home_distance) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_home_distance = new_osd_home_distance;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_osd_got_home() {
  uint8_t TEMP_osd_got_home = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_got_home = adhoc_osd_state.osd_got_home;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_got_home;
}

void set_osd_got_home(uint8_t new_osd_got_home) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_got_home = new_osd_got_home;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_osd_alt_cnt() {
  uint8_t TEMP_osd_alt_cnt = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_alt_cnt = adhoc_osd_state.osd_alt_cnt;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_alt_cnt;
}

void set_osd_alt_cnt(uint8_t new_osd_alt_cnt) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_alt_cnt = new_osd_alt_cnt;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_osd_alt_prev() {
  uint8_t TEMP_osd_alt_prev = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_alt_prev = adhoc_osd_state.osd_alt_prev;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_alt_prev;
}

void set_osd_alt_prev(uint8_t new_osd_alt_prev) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_alt_prev = new_osd_alt_prev;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_osd_home_alt() {
  uint8_t TEMP_osd_home_alt = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_osd_home_alt = adhoc_osd_state.osd_home_alt;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_osd_home_alt;
}

void set_osd_home_alt(uint8_t new_osd_home_alt) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.osd_home_alt = new_osd_home_alt;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_current_panel() {
  uint8_t TEMP_current_panel = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_current_panel = adhoc_osd_state.current_panel;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_current_panel;
}

void set_current_panel(uint8_t new_current_panel) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.current_panel = new_current_panel;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_mavbeat() {
  uint8_t TEMP_mavbeat = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_mavbeat = adhoc_osd_state.mavbeat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_mavbeat;
}

void set_mavbeat(uint8_t new_mavbeat) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.mavbeat = new_mavbeat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_lastMAVBeat() {
  uint32_t TEMP_lastMAVBeat = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_lastMAVBeat = adhoc_osd_state.lastMAVBeat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_lastMAVBeat;
}

void set_lastMAVBeat(uint32_t new_lastMAVBeat) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.lastMAVBeat = new_lastMAVBeat;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_lastWritePanel() {
  uint32_t TEMP_lastWritePanel = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_lastWritePanel = adhoc_osd_state.lastWritePanel;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_lastWritePanel;
}

void set_lastWritePanel(uint32_t new_lastWritePanel) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.lastWritePanel = new_lastWritePanel;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_waitingMAVBeats() {
  uint8_t TEMP_waitingMAVBeats = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_waitingMAVBeats = adhoc_osd_state.waitingMAVBeats;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_waitingMAVBeats;
}

void set_waitingMAVBeats(uint8_t new_waitingMAVBeats) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.waitingMAVBeats = new_waitingMAVBeats;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_mav_type() {
  uint8_t TEMP_mav_type = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_mav_type = adhoc_osd_state.mav_type;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_mav_type;
}

void set_mav_type(uint8_t new_mav_type) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.mav_type = new_mav_type;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_mav_system() {
  uint8_t TEMP_mav_system = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_mav_system = adhoc_osd_state.mav_system;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_mav_system;
}

void set_mav_system(uint8_t new_mav_system) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.mav_system = new_mav_system;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_mav_component() {
  uint8_t TEMP_mav_component = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_mav_component = adhoc_osd_state.mav_component;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_mav_component;
}

void set_mav_component(uint8_t new_mav_component) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.mav_component = new_mav_component;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint8_t get_enable_mav_request() {
  uint8_t TEMP_enable_mav_request = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_enable_mav_request = adhoc_osd_state.enable_mav_request;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_enable_mav_request;
}

void set_enable_mav_request(uint8_t new_enable_mav_request) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.enable_mav_request = new_enable_mav_request;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_sys_start_time() {
  uint32_t TEMP_sys_start_time = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_sys_start_time = adhoc_osd_state.sys_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_sys_start_time;
}

void set_sys_start_time(uint32_t new_sys_start_time) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.sys_start_time = new_sys_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_heartbeat_start_time() {
  uint32_t TEMP_heartbeat_start_time = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_heartbeat_start_time = adhoc_osd_state.heartbeat_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_heartbeat_start_time;
}

void set_heartbeat_start_time(uint32_t new_heartbeat_start_time) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.heartbeat_start_time = new_heartbeat_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_armed_start_time() {
  uint32_t TEMP_armed_start_time = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_armed_start_time = adhoc_osd_state.armed_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_armed_start_time;
}

void set_armed_start_time(uint32_t new_armed_start_time) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.armed_start_time = new_armed_start_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}

uint32_t get_total_armed_time() {
  uint32_t TEMP_total_armed_time = 0;
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      TEMP_total_armed_time = adhoc_osd_state.total_armed_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return TEMP_total_armed_time;
}

void set_total_armed_time(uint32_t new_total_armed_time) {
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      adhoc_osd_state.total_armed_time = new_total_armed_time;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
}


