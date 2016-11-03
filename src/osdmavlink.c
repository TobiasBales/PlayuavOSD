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

#include "osdmavlink.h"
#include "osdvar.h"
#include "osdconfig.h"

#define MAX_STREAMS 6

mavlink_system_t mavlink_system = { 7, 1 }; //Ardupilot:7,1  Pixhawk:100,50
mavlink_message_t msg;
mavlink_status_t status;
uint8_t mavlink_active = 0;
uint8_t mavlink_requested = 0;

extern xSemaphoreHandle onMavlinkSemaphore;
extern uint8_t *mavlink_buffer_proc;

// This is the OSD state that the Mavlink thread owns
osd_state mavlink_osd_state = {};

// This mutex controls access to the Mavlink OSD State
xSemaphoreHandle osd_state_mavlink_mutex;

void request_mavlink_rates(void) {
  const u8 MAVStreams[MAX_STREAMS] = { MAV_DATA_STREAM_RAW_SENSORS,
                                       MAV_DATA_STREAM_EXTENDED_STATUS,
                                       MAV_DATA_STREAM_RC_CHANNELS,
                                       MAV_DATA_STREAM_POSITION,
                                       MAV_DATA_STREAM_EXTRA1,
                                       MAV_DATA_STREAM_EXTRA2 };
  //uint16_t MAVRates[MAX_STREAMS] = {0x01, 0x02, 0x05, 0x02, 0x05, 0x02};
  uint16_t MAVRates[MAX_STREAMS] = { 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A };

//	if(mav_component == 0x32) //pixhawk origin FW
//	{
//		return; //we need not change the rate
//	}
  for (uint32_t i = 0; i < MAX_STREAMS; i++) {
    mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
                                         get_mav_system(), get_mav_component(),
                                         MAVStreams[i], MAVRates[i], 1);
  }
}

void request_mission_count(void) {
  mavlink_msg_mission_request_list_send(MAVLINK_COMM_0,
                                        get_mav_system(), get_mav_component());
}

void request_mission_item(uint16_t index) {
  mavlink_msg_mission_request_send(MAVLINK_COMM_0,
                                   get_mav_system(), get_mav_component(), index);
}

void parseMavlink(void) {
  uint8_t c;
  uint8_t index = 0;
  uint8_t mavtype = 0;

  while (index < MAVLINK_BUFFER_SIZE)
  {
    c = mavlink_buffer_proc[index++];
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      mavlink_active = 1;
      //handle msg
      switch (msg.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT:
      {
        if ((msg.compid != 1) && (msg.compid != 50)) {
          // MAVMSG not from ardupilot(component ID:1) or pixhawk(component ID:50)
          break;
        }

        mavtype = mavlink_msg_heartbeat_get_type(&msg);
        if (mavtype == 6) {
          // MAVMSG from GCS
          break;
        }

        set_mavbeat(1);
        set_mav_system(msg.sysid);
        set_mav_component(msg.compid);
        set_mav_type(mavtype);
        mavlink_osd_state.autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
        mavlink_osd_state.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
        mavlink_osd_state.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);

        mavlink_osd_state.last_motor_armed = mavlink_osd_state.motor_armed;
        mavlink_osd_state.motor_armed = mavlink_osd_state.base_mode & (1 << 7);

        if (get_heartbeat_start_time() == 0) {
          set_heartbeat_start_time(GetSystimeMS());
        }

        if (!mavlink_osd_state.last_motor_armed && mavlink_osd_state.motor_armed) {
          set_armed_start_time(GetSystimeMS());
        }

        if (mavlink_osd_state.last_motor_armed && !mavlink_osd_state.motor_armed) {
          set_total_armed_time(GetSystimeMS() - get_armed_start_time() + get_total_armed_time());
          set_armed_start_time(0);
        }

        set_lastMAVBeat(GetSystimeMS());
        if (get_waitingMAVBeats() == 1) {
          set_enable_mav_request(1);
        }

        // If we've never gotten mission_counts, and we aren't currently asking for them, 
        // request the mission_count
        if ((mavlink_osd_state.got_mission_counts == 0) && (mavlink_osd_state.enable_mission_count_request == 0))
        {
          mavlink_osd_state.enable_mission_count_request = 1;
        }
      }
      break;
      case MAVLINK_MSG_ID_SYS_STATUS:
      {
        mavlink_osd_state.osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f);                 //Battery voltage, in millivolts (1 = 1 millivolt)
        mavlink_osd_state.osd_curr_A = mavlink_msg_sys_status_get_current_battery(&msg);                 //Battery current, in 10*milliamperes (1 = 10 milliampere)
        mavlink_osd_state.osd_battery_remaining_A = mavlink_msg_sys_status_get_battery_remaining(&msg);                 //Remaining battery energy: (0%: 0, 100%: 100)
        //custom_mode = mav_component;//Debug
        //osd_nav_mode = mav_system;//Debug
      }
      break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
      {
        float osd_lat_new = mavlink_msg_gps_raw_int_get_lat(&msg);
        float osd_lon_new = mavlink_msg_gps_raw_int_get_lon(&msg);
        
        mavlink_osd_state.osd_lat = osd_lat_new;
        mavlink_osd_state.osd_lon = osd_lon_new;
        
        mavlink_osd_state.osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
        mavlink_osd_state.osd_hdop = mavlink_msg_gps_raw_int_get_eph(&msg);
        mavlink_osd_state.osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
      }
      break;
      case MAVLINK_MSG_ID_GPS2_RAW:
      {
        mavlink_osd_state.osd_lat2 = mavlink_msg_gps2_raw_get_lat(&msg);
        mavlink_osd_state.osd_lon2 = mavlink_msg_gps2_raw_get_lon(&msg);
        mavlink_osd_state.osd_fix_type2 = mavlink_msg_gps2_raw_get_fix_type(&msg);
        mavlink_osd_state.osd_hdop2 = mavlink_msg_gps2_raw_get_eph(&msg);
        mavlink_osd_state.osd_satellites_visible2 = mavlink_msg_gps2_raw_get_satellites_visible(&msg);
      }
      break;
      case MAVLINK_MSG_ID_VFR_HUD:
      {
        mavlink_osd_state.osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
        mavlink_osd_state.osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
        mavlink_osd_state.osd_heading = mavlink_msg_vfr_hud_get_heading(&msg);                 // 0..360 deg, 0=north
        mavlink_osd_state.osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
        //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
        //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
        mavlink_osd_state.osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
        mavlink_osd_state.osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
      }
      break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:                        // jmmods
      {
        mavlink_osd_state.osd_rel_alt = mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000.0;                    // jmmods
        // uav.relative_alt = packet.relative_alt / 1000.0; //jmtune Altitude above ground in meters, expressed as * 1000 (millimeters)   // jmmods
      }
      break;

      case MAVLINK_MSG_ID_ATTITUDE:
      {
        mavlink_osd_state.osd_pitch = Rad2Deg(mavlink_msg_attitude_get_pitch(&msg));
        mavlink_osd_state.osd_roll = Rad2Deg(mavlink_msg_attitude_get_roll(&msg));
        mavlink_osd_state.osd_yaw = Rad2Deg(mavlink_msg_attitude_get_yaw(&msg));
      }
      break;
      case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
      {
        mavlink_osd_state.nav_roll = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
        mavlink_osd_state.nav_pitch = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
        mavlink_osd_state.nav_bearing = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
        mavlink_osd_state.wp_target_bearing = mavlink_msg_nav_controller_output_get_target_bearing(&msg);
        mavlink_osd_state.wp_dist = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
        mavlink_osd_state.alt_error = mavlink_msg_nav_controller_output_get_alt_error(&msg);
        mavlink_osd_state.aspd_error = mavlink_msg_nav_controller_output_get_aspd_error(&msg);
        mavlink_osd_state.xtrack_error = mavlink_msg_nav_controller_output_get_xtrack_error(&msg);
      }
      break;
      case MAVLINK_MSG_ID_MISSION_CURRENT:
      {
        mavlink_osd_state.wp_number = (uint8_t)mavlink_msg_mission_current_get_seq(&msg);
      }
      break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
      {
        if (!mavlink_osd_state.osd_chan_cnt_above_eight)
        {
          mavlink_osd_state.osd_chan1_raw = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
          mavlink_osd_state.osd_chan2_raw = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
          mavlink_osd_state.osd_chan3_raw = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
          mavlink_osd_state.osd_chan4_raw = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
          mavlink_osd_state.osd_chan5_raw = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
          mavlink_osd_state.osd_chan6_raw = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
          mavlink_osd_state.osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
          mavlink_osd_state.osd_chan8_raw = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
          mavlink_osd_state.osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
        }
      }
      break;
      case MAVLINK_MSG_ID_RC_CHANNELS:
      {
        mavlink_osd_state.osd_chan_cnt_above_eight = true;
        mavlink_osd_state.osd_chan1_raw = mavlink_msg_rc_channels_get_chan1_raw(&msg);
        mavlink_osd_state.osd_chan2_raw = mavlink_msg_rc_channels_get_chan2_raw(&msg);
        mavlink_osd_state.osd_chan3_raw = mavlink_msg_rc_channels_get_chan3_raw(&msg);
        mavlink_osd_state.osd_chan4_raw = mavlink_msg_rc_channels_get_chan4_raw(&msg);
        mavlink_osd_state.osd_chan5_raw = mavlink_msg_rc_channels_get_chan5_raw(&msg);
        mavlink_osd_state.osd_chan6_raw = mavlink_msg_rc_channels_get_chan6_raw(&msg);
        mavlink_osd_state.osd_chan7_raw = mavlink_msg_rc_channels_get_chan7_raw(&msg);
        mavlink_osd_state.osd_chan8_raw = mavlink_msg_rc_channels_get_chan8_raw(&msg);
        mavlink_osd_state.osd_chan9_raw = mavlink_msg_rc_channels_get_chan9_raw(&msg);
        mavlink_osd_state.osd_chan10_raw = mavlink_msg_rc_channels_get_chan10_raw(&msg);
        mavlink_osd_state.osd_chan11_raw = mavlink_msg_rc_channels_get_chan11_raw(&msg);
        mavlink_osd_state.osd_chan12_raw = mavlink_msg_rc_channels_get_chan12_raw(&msg);
        mavlink_osd_state.osd_chan13_raw = mavlink_msg_rc_channels_get_chan13_raw(&msg);
        mavlink_osd_state.osd_chan14_raw = mavlink_msg_rc_channels_get_chan14_raw(&msg);
        mavlink_osd_state.osd_chan15_raw = mavlink_msg_rc_channels_get_chan15_raw(&msg);
        mavlink_osd_state.osd_chan16_raw = mavlink_msg_rc_channels_get_chan16_raw(&msg);
        mavlink_osd_state.osd_rssi = mavlink_msg_rc_channels_get_rssi(&msg);
      }
      break;
      case MAVLINK_MSG_ID_WIND:
      {
        mavlink_osd_state.osd_windDir = mavlink_msg_wind_get_direction(&msg);                 // 0..360 deg, 0=north
        mavlink_osd_state.osd_windSpeed = mavlink_msg_wind_get_speed(&msg);                 //m/s
      }
      break;
      
      case MAVLINK_MSG_ID_MISSION_COUNT:
      {
        mavlink_osd_state.mission_counts = mavlink_msg_mission_count_get_count(&msg);
        mavlink_osd_state.got_mission_counts = 1;
        mavlink_osd_state.enable_mission_item_request = 1;
        mavlink_osd_state.current_mission_item_req_index = 0;
        mavlink_osd_state.wp_counts = 0;
      }
      break;

      case MAVLINK_MSG_ID_MISSION_ITEM:
      {
        uint16_t seq, cmd;

        seq = mavlink_msg_mission_item_get_seq(&msg);
        cmd = mavlink_msg_mission_item_get_command(&msg);

        // received a packet, but not what we requested
        if (mavlink_osd_state.current_mission_item_req_index == seq)
        {
          //store the waypoints
          if ((cmd == MAV_CMD_NAV_WAYPOINT) && (mavlink_osd_state.wp_counts < MAX_WAYPOINTS))
          {
            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].seq = seq;
            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].cmd = cmd;

            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].x = mavlink_msg_mission_item_get_x(&msg);
            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].y = mavlink_msg_mission_item_get_y(&msg);
            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].z = mavlink_msg_mission_item_get_z(&msg);

            mavlink_osd_state.wp_list[mavlink_osd_state.wp_counts].current = mavlink_msg_mission_item_get_current(&msg);
            mavlink_osd_state.wp_counts++;
          }

          mavlink_osd_state.current_mission_item_req_index++;
          if (mavlink_osd_state.current_mission_item_req_index >= mavlink_osd_state.mission_counts) {
            mavlink_osd_state.enable_mission_item_request = 0;
            mavlink_osd_state.got_all_wps = 1;
          }
        }



      }
      break;

/*
                // will be used in the future. See Samuel's PR:https://github.com/PlayUAV/PlayuavOSD/pull/13
                // Noticed: the type of variable in this message is int32_t. Currently we use float type to simulate.
                case MAVLINK_MSG_ID_BATTERY_STATUS:
                   {
                       osd_battery_consumed_in_mah = mavlink_msg_battery_status_get_current_consumed(&msg);
                   }
                   break;
 */
      default:
        //Do nothing
        break;
      }       //end switch(msg.msgid)
    }
  }
}

void copyNewMavlinkValuesToAirlock() {
    // Mavlink is presumed to be the slower thread, and more tolerant of delays. Therefore
    // we use a large timeout for the Mutex.
    copy_osd_state(&mavlink_osd_state, &airlock_osd_state, portMAX_DELAY);
}

void MavlinkTask(void *pvParameters) {
  clear_osd_state_struct(&mavlink_osd_state);
  mavlink_usart_init(get_map_bandrate(eeprom_buffer.params.uart_bandrate));    // jmmods 19200 for ultimate lrs use
  set_sys_start_time(GetSystimeMS());

  while (1)
  {
    // Take event semaphore
    xSemaphoreTake(onMavlinkSemaphore, portMAX_DELAY);
    // Lock access to Mavlink OSDState via mutex
    if (xSemaphoreTake(osd_state_mavlink_mutex, portMAX_DELAY) == pdTRUE ) {

        // Do Mavlink parsing from serial input
        parseMavlink();
        // Copy updated values into the airlock
        copyNewMavlinkValuesToAirlock();

        // Release the Mavlink OSDState mutex
        xSemaphoreGive(osd_state_mavlink_mutex);
    }   
    // At this point Mavlink OSDState mutex is briefly open for access by others (i.e. Board.c)
  }
}
