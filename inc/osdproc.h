#ifndef OSD_PROC_H__
#define OSD_PROC_H__

#include "board.h"

/// GPS status codes
enum GPS_Status {
  NO_GPS = 0,                     ///< No GPS connected/detected
  NO_FIX = 1,                     ///< Receiving valid GPS messages but no lock
  GPS_OK_FIX_2D = 2,              ///< Receiving valid messages and 2D lock
  GPS_OK_FIX_3D = 3,              ///< Receiving valid messages and 3D lock
  GPS_OK_FIX_3D_DGPS = 4,         ///< Receiving valid messages and 3D lock with differential improvements
  GPS_OK_FIX_3D_RTK_FLOAT = 5,    ///< Receiving valid messages and 3D lock, with relative-positioning improvements (RTK Float, 3D Position)
  GPS_OK_FIX_TYPE_RTK_FIXED = 6,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements (RTK Fixed, 3D Position)
  GPS_OK_FIX_TYPE_STATIC = 7      ///< Static fixed, typically used for base stations  
};

void vTaskOSD(void *pvParameters);

void RenderScreen(void);

void set_home_position_if_unset(void);
void set_home_altitude_if_unset(void);  
void set_home_distance_and_bearing(void);

void draw_distance_to_home(void);
void draw_distance_to_waypoint(void);

void draw_uav3d(void);
void draw_uav2d(void);
void draw_throttle(void);
void draw_simple_attitude(void);
void draw_home_direction(void);
void draw_home_direction_debug_info(int x, int y, float absolute_home_bearing, float uav_compass_bearing, float relative_home_bearing);
void draw_radar(void);
void draw_flight_mode(void);
void draw_arm_state(void);
void draw_battery_voltage(void);
void draw_battery_current(void);
void draw_battery_remaining(void);
void draw_battery_consumed(void);
void draw_altitude_scale(void);
void draw_absolute_altitude(void);
void draw_relative_altitude(void);
void draw_speed_scale(void);
void draw_ground_speed(void);
void draw_air_speed(void);
void draw_home_latitude(void);
void draw_home_longitude(void);
void draw_gps_status(void);
void draw_gps_hdop(void);
void draw_gps_latitude(void);
void draw_gps_longitude(void);
void draw_gps2_status(void);
void draw_gps2_hdop(void);
void draw_gps2_latitude(void);
void draw_gps2_longitude(void);
void draw_total_trip(void);
void draw_time(void);
void draw_climb_rate(void);
void draw_rssi(void);
void draw_link_quality(void);
void draw_efficiency(void);
void draw_rc_channels(void);
void draw_wind(void);
void draw_map(void);
void draw_panel_changed(void);
void draw_warning(void);
void draw_head_wp_home(void);
void draw_watts(void);
void draw_osd_linear_compass(void);
void draw_version_splash(void);

void draw_vertical_scale(int v, int range, int halign, int x, int y, int height, int mintick_step,
                         int majtick_step, int mintick_len, int majtick_len,
                         int boundtick_len, __attribute__((unused)) int max_val, int flags);

void draw_linear_compass(int v, int home_dir, int range, int width, int x, int y, int mintick_step,
                         int majtick_step, int mintick_len, int majtick_len,
                         __attribute__((unused)) int flags);
                         
float get_bearing_to_home_in_degrees(void);
float get_distance_from_home_in_meters(void);

void DJI_test(void);

#endif //OSD_PROC_H__
