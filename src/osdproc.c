/*
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
 * Tau Labs - Brain FPV Flight Controller(https://github.com/BrainFPV/TauLabs)
 */
#include "osdproc.h"
#include "graphengine.h"
#include "led.h"
#include "osdcore.h"
#include "osdvar.h"
#include "fonts.h"
#include "UAVObj.h"
#include "osdconfig.h"
#include "osdmavlink.h"
#include "math3d.h"
#include "px4_custom_mode.h"

#define HUD_VSCALE_FLAG_CLEAR       1
#define HUD_VSCALE_FLAG_NO_NEGATIVE 2

extern xSemaphoreHandle onScreenDisplaySemaphore;

extern xSemaphoreHandle osd_state_adhoc_mutex;

// This is the OSD state that the OSDProc thread owns
osd_state osdproc_osd_state = {};

// This mutex controls access to the OSDProc OSD State
xSemaphoreHandle osd_state_osdproc_mutex;

int32_t test_alt, test_speed, test_throttle;

//2:small, 0:normal, 3:large
const int SIZE_TO_FONT[3] = { 2, 0, 3 };

// The bool is because I fear overflow on the milliseconds elapsed calculation
bool version_splash_shown = false;
int32_t version_splash_start_time = 0;

uint8_t last_panel = 1;
int32_t new_panel_start_time = 0;

uint8_t last_warn_type = 0;
int32_t last_warn_time = 0;
char* warn_str = "";

const char METRIC_SPEED[] = "KM/H";         //kilometer per hour
const char METRIC_DIST_SHORT[] = "M";       //meter
const char METRIC_DIST_LONG[] = "KM";       //kilometer

const char IMPERIAL_SPEED[] = "M/H";        //mile per hour
const char IMPERIAL_DIST_SHORT[] = "F";     //feet
const char IMPERIAL_DIST_LONG[] = "M";      //mile

// Unit conversion constants
float convert_speed = 0.0f;
float convert_distance = 0.0f;
float convert_distance_divider = 0.0f;
const char * dist_unit_short = METRIC_DIST_SHORT;
const char * dist_unit_long = METRIC_DIST_LONG;
const char * spd_unit = METRIC_SPEED;

extern uint8_t *write_buffer_tele;
void debug_wps();

static void inline setbit(uint8_t* buf, uint32_t bit) {
  uint32_t byte_index = bit / 8U;
  uint8_t bit_pos = bit % 8U;
  buf[byte_index] |= (1U << (7U - bit_pos));
}

static inline void clearbit(uint8_t* buf, uint32_t bit) {
  uint32_t byte_index = bit / 8U;
  uint8_t bit_pos = bit % 8U;
  buf[byte_index] &= ~(1U << (7U - bit_pos));
}

// should be 9 bytes per line
#define  TELEM_DATA_BYTES_PER_LINE ((TELEM_BUFFER_WIDTH * 8U) / 10U)


/*
   Assumes that a 0 in the low level video dma buffer write_buffer_tele represents a mark state
   and a 1 represents a space state
   and that ar length >= TELEM_LINES * TELEM_DATA_BYTES_PER_LINE
 */
void write_data(uint8_t const * ar) {

  memset(write_buffer_tele, 0, TELEM_LINES * TELEM_BUFFER_WIDTH);

  for (uint32_t y = 0, yend = TELEM_LINES; y < yend; ++y) {   // rows
    // start of line mark state
    uint32_t bit_offset = y * 8U * TELEM_BUFFER_WIDTH + 3U;
    for (uint32_t xbyte = 0, xend = TELEM_DATA_BYTES_PER_LINE; xbyte < xend; ++xbyte) {  // columns
      // start bit
      setbit(write_buffer_tele, bit_offset);
      ++bit_offset;
      uint8_t const cur_val = *ar;
      for (uint32_t bitpos = 0U; bitpos < 8U; ++bitpos) {
        if ( (cur_val & (1U << bitpos)) == 0U) {
          setbit(write_buffer_tele, bit_offset);
        }
        ++bit_offset;
      }
      // stop bit
      ++bit_offset;
      ++ar;
    }
    // rest of line mark state

  }
}

// the user layer buffer
// user can write 8 bit values for transmission here
// TODO change to uint8_t and test
static char telem_tx_buffer[TELEM_LINES * TELEM_DATA_BYTES_PER_LINE] = { 0 };

void VBI_test(void) {
  uint32_t time_now = GetSystimeMS();

  snprintf(telem_tx_buffer, TELEM_LINES * TELEM_DATA_BYTES_PER_LINE, "PlayUAV time = %u", time_now);

  write_data(telem_tx_buffer);
}

void do_converts(void) {
  if (eeprom_buffer.params.Units_mode == 1)
  {
    convert_distance = 3.28f;
    convert_speed = 2.23f;
    convert_distance_divider = 5280.0f;     // feet in a mile
    dist_unit_short = IMPERIAL_DIST_SHORT;
    dist_unit_long = IMPERIAL_DIST_LONG;
    spd_unit = IMPERIAL_SPEED;
  }
  else
  {
    convert_distance = 1.0f;
    convert_speed = 3.6f;
    convert_distance_divider = 1000.0f;
    dist_unit_short = METRIC_DIST_SHORT;
    dist_unit_long = METRIC_DIST_LONG;
    spd_unit = METRIC_SPEED;
  }
}

bool shownAtPanel(uint16_t itemPanel) {
  bool result = false;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      //issue #1 - fixed
      result = ((itemPanel & (1 << (adhoc_osd_state.current_panel - 1))) != 0);
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  return result;
}

bool enabledAndShownOnPanel(uint16_t enabled, uint16_t panel) {
  return enabled == 1 && shownAtPanel(panel);
}

void copyNewAirlockValuesToOsdProc() {
    // Considered using a deliberately short delay here, but it seems ok this way.
    copy_osd_state(&airlock_osd_state, &osdproc_osd_state, portMAX_DELAY);
}

void getOsdXandYOffsets(int8_t * p_osd_offset_X, int8_t * p_osd_offset_Y) {
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      // Get the OSD offsets
      *p_osd_offset_X = adhoc_osd_state.osd_offset_X;
      *p_osd_offset_Y = adhoc_osd_state.osd_offset_Y;
      // Release the ad-hoc mutex      
      xSemaphoreGive(osd_state_adhoc_mutex);
  }        
}

void setOsdOffsets(){
  int8_t TEMP_osd_offset_X;
  int8_t TEMP_osd_offset_Y;  
  
  getOsdXandYOffsets(&TEMP_osd_offset_X, &TEMP_osd_offset_Y);  
  
  osdVideoSetXOffset(TEMP_osd_offset_X);
  osdVideoSetYOffset(TEMP_osd_offset_Y); 
}

void vTaskOSD(void *pvParameters) {
    
  clear_osd_state_struct(&osdproc_osd_state);    
    
  uav3D_init();
  uav2D_init();
  simple_attitude_init();
  home_direction_init();

  setOsdOffsets();  
  osdCoreInit();

  while (1)
  {
    xSemaphoreTake(onScreenDisplaySemaphore, portMAX_DELAY);
    
    // Lock access to OSDProc OSD state via mutex
    if (xSemaphoreTake(osd_state_osdproc_mutex, portMAX_DELAY) == pdTRUE ) {
    
        copyNewAirlockValuesToOsdProc();
        clearGraphics();
        RenderScreen();

        // Release the Mavlink OSDState mutex
        xSemaphoreGive(osd_state_osdproc_mutex);
    }
    // At this point OSDProc OSDState mutex is briefly open for access by others, should
    // they need it. 
  }
}

void resetPanelNumberIfBadPanelValue(){
  // Get ad-hoc mutex
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      // Reset panel to #1 if the panel number is out of range
      if (adhoc_osd_state.current_panel < 1 || adhoc_osd_state.current_panel > eeprom_buffer.params.Max_panels) {
        adhoc_osd_state.current_panel = 1;
      }
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
    }  
}

// TODO: try if this is performance critical or not
char tmp_str[50] = { 0 };
char* tmp_str1 = "";

void RenderScreen(void) {
  do_converts();

  resetPanelNumberIfBadPanelValue();

  set_home_position_if_unset();
  set_home_altitude_if_unset();

  set_home_distance_and_bearing();

  draw_flight_mode();
  draw_arm_state();
  draw_battery_voltage();
  draw_battery_current();
  draw_battery_remaining();
  draw_battery_consumed();
  draw_altitude_scale();
  draw_absolute_altitude();
  draw_relative_altitude();
  draw_speed_scale();
  draw_ground_speed();
  draw_air_speed();
  draw_home_direction();
  draw_uav3d();
  draw_uav2d();
  draw_throttle();
  draw_home_latitude();
  draw_home_longitude();
  draw_gps_status();
  draw_gps_hdop();
  draw_gps_latitude();
  draw_gps_longitude();
  draw_gps2_status();
  draw_gps2_hdop();
  draw_gps2_latitude();
  draw_gps2_longitude();
  draw_total_trip();
  draw_time();
  draw_distance_to_home();
  draw_distance_to_waypoint();
  draw_head_wp_home();
  draw_osd_linear_compass();
  draw_climb_rate();
  draw_rssi();
  draw_link_quality();
  draw_efficiency();
  draw_watts();
  draw_wind();
  draw_map();
  draw_rc_channels();

  draw_warning();  
  draw_panel_changed();
  
  draw_version_splash();
}

int get_map_radius() {
  int map_radius = 0;
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      map_radius = (int)(eeprom_buffer.params.Atti_3D_map_radius * adhoc_osd_state.atti_3d_scale);
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
   }
      
   return map_radius;
}

void draw_uav3d(void) {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Atti_3D_en,
                              eeprom_buffer.params.Atti_3D_panel)) {
    return;
  }

  static MATRIX4X4 mrot;   // general rotation matrix

  static int32_t roll = 0;
  static int32_t pitch = 0;
  static int32_t yaw = 0;
  int x = eeprom_buffer.params.Atti_3D_posX;
  int y = eeprom_buffer.params.Atti_3D_posY;
  int map_radius = get_map_radius();

  write_string("N", x, y - map_radius, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);
  write_string("E", x + map_radius, y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);
  //need to adjust viewport base on the video mode
  Adjust_Viewport_CAM4DV1(&cam, GRAPHICS_RIGHT, GRAPHICS_BOTTOM);

  roll = ((int32_t)osdproc_osd_state.osd_roll + 360) % 360;
  roll = fabs(roll - 360);

  pitch = ((int32_t)osdproc_osd_state.osd_pitch + 360) % 360;
  pitch = fabs(pitch - 360);

  yaw = fabs(osdproc_osd_state.osd_heading - 360);

  Reset_OBJECT4DV1(&uav3D);

  // generate rotation matrix around
  Build_XYZ_Rotation_MATRIX4X4(pitch, roll, yaw, &mrot);

  // rotate and transfer to world coord
  Transform_To_World_OBJECT4DV1(&uav3D, &mrot);

  // generate camera matrix
  Build_CAM4DV1_Matrix_Euler(&cam);

  // transfer obj world -> camera -> perspective -> screen
  // then perform backfaces remove
  Transform_To_Screen_OBJECT4DV1(&uav3D, &cam);

  //draw object wire
  for (int poly = 0; poly < uav3D.num_polys; poly++)
  {
    // render this polygon if and only if it's not clipped, not culled,
    // active, and visible, note however the concecpt of "backface" is
    // irrelevant in a wire frame engine though
    if (!(uav3D.plist[poly].state & POLY4DV1_STATE_ACTIVE) ||
        (uav3D.plist[poly].state & POLY4DV1_STATE_CLIPPED) ||
        (uav3D.plist[poly].state & POLY4DV1_STATE_BACKFACE) )
      continue;      // move onto next poly

    // extract vertex indices into master list, rember the polygons are
    // NOT self contained, but based on the vertex list stored in the object
    // itself
    int vindex_0 = uav3D.plist[poly].vert[0];
    int vindex_1 = uav3D.plist[poly].vert[1];
    int vindex_2 = uav3D.plist[poly].vert[2];

    // draw the lines now
    write_line_outlined(uav3D.vlist_trans[vindex_0].x, uav3D.vlist_trans[vindex_0].y,
                        uav3D.vlist_trans[vindex_1].x, uav3D.vlist_trans[vindex_1].y,
                        2, 2, 0, 1);

    write_line_outlined(uav3D.vlist_trans[vindex_1].x, uav3D.vlist_trans[vindex_1].y,
                        uav3D.vlist_trans[vindex_2].x, uav3D.vlist_trans[vindex_2].y,
                        2, 2, 0, 1);

    write_line_outlined(uav3D.vlist_trans[vindex_2].x, uav3D.vlist_trans[vindex_2].y,
                        uav3D.vlist_trans[vindex_0].x, uav3D.vlist_trans[vindex_0].y,
                        2, 2, 0, 1);

  }   // end for poly
}

void draw_simple_attitude() {
  Reset_Polygon2D(&simple_attitude);
  int pitch = osdproc_osd_state.osd_pitch;
  const int max_pitch = 60;
  const int radius = 4 * get_atti_mp_scale();
  const int x = simple_attitude.x0;
  const int y = simple_attitude.y0;
  if (pitch > max_pitch) {
    pitch = max_pitch;
  } else if (pitch < -max_pitch) {
    pitch = -max_pitch;
  }

  write_line_outlined(x - radius - 1, y, x - 2 * radius - 1, y, 0, 0, 0, 1);
  write_line_outlined(x + radius - 1, y, x + 2 * radius + 1, y, 0, 0, 0, 1);
  write_line_outlined(x, y - radius - 1, x, y - 2 * radius, 0, 0, 0, 1);
  write_circle_outlined(x, y, radius, 0, 1, 0, 1);

  Transform_Polygon2D(&simple_attitude, -osdproc_osd_state.osd_roll, 0, pitch);
  VECTOR4D v;
  for (int i = 0; i < simple_attitude.num_verts; i += 2) {
    write_line_outlined(simple_attitude.vlist_trans[i].x + x, simple_attitude.vlist_trans[i].y + y,
                        simple_attitude.vlist_trans[i + 1].x + x, simple_attitude.vlist_trans[i + 1].y + y,
                        2, 2, 0, 1);
  }
}


void draw_radar() {
  int index = 0;

  Reset_Polygon2D(&uav2D);
  Transform_Polygon2D(&uav2D, -osdproc_osd_state.osd_roll, 0, osdproc_osd_state.osd_pitch);

  // loop thru and draw a line from vertices 1 to n
  VECTOR4D v;
  for (index = 0; index < uav2D.num_verts - 1; )
  {
    VECTOR4D_INITXYZW(&v,   uav2D.vlist_trans[index].x + uav2D.x0, uav2D.vlist_trans[index].y + uav2D.y0,
                      uav2D.vlist_trans[index + 1].x + uav2D.x0, uav2D.vlist_trans[index + 1].y + uav2D.y0);
    if (Clip_Line(&v))
    {
      write_line_outlined(v.x, v.y, v.z, v.w, 2, 2, 0, 1);
    }
    index += 2;
  }   // end for

  //rotate roll scale and display, we only cal x
  Reset_Polygon2D(&rollscale2D);
  Rotate_Polygon2D(&rollscale2D, -osdproc_osd_state.osd_roll);
  for (index = 0; index < rollscale2D.num_verts - 1; index++)
  {
    // draw line from ith to ith+1 vertex
    write_line_outlined(rollscale2D.vlist_trans[index].x + rollscale2D.x0, rollscale2D.vlist_trans[index].y + rollscale2D.y0,
                        rollscale2D.vlist_trans[index + 1].x + rollscale2D.x0, rollscale2D.vlist_trans[index + 1].y + rollscale2D.y0,
                        2, 2, 0, 1);
  }   // end for

  int x = eeprom_buffer.params.Atti_mp_posX;
  int y = eeprom_buffer.params.Atti_mp_posY;
  float atti_mp_scale = get_atti_mp_scale();
  int wingStart = (int)(12.0f * atti_mp_scale);
  int wingEnd = (int)(7.0f * atti_mp_scale);
  char tmp_str[10] = { 0 };
  //draw uav
  write_line_outlined(x, y, x - 9, y + 5, 2, 2, 0, 1);
  write_line_outlined(x, y, x + 9, y + 5, 2, 2, 0, 1);
  write_line_outlined(x - wingStart, y, x - wingEnd, y, 2, 2, 0, 1);
  write_line_outlined(x + wingEnd, y, x + wingStart, y, 2, 2, 0, 1);


  write_filled_rectangle_lm(x - 9, y + 6, 15, 9, 0, 1);
  sprintf(tmp_str, "%d", (int)osdproc_osd_state.osd_pitch);
  write_string(tmp_str, x, y + 5, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);

  y = eeprom_buffer.params.Atti_mp_posY - (int)(38.0f * atti_mp_scale);
  //draw roll value
  write_line_outlined(x, y, x - 4, y + 8, 2, 2, 0, 1);
  write_line_outlined(x, y, x + 4, y + 8, 2, 2, 0, 1);
  write_line_outlined(x - 4, y + 8, x + 4, y + 8, 2, 2, 0, 1);
  sprintf(tmp_str, "%d", (int)osdproc_osd_state.osd_roll);
  write_string(tmp_str, x, y - 3, 0, 0, TEXT_VA_BOTTOM, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);
}




void draw_home_direction() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.HomeDirection_enabled,
                              eeprom_buffer.params.HomeDirection_panel)) {
    return;
  }

  // The absolute (0-360, 0 being north) bearing to home, 
  // irrespective of the direction the UAV is oriented.
  float absolute_home_bearing = get_osd_home_bearing();
  // The UAV's current compass bearing (0-360, 0 being north). 
  // This is the direction the UAV is facing.
  float uav_compass_bearing = osdproc_osd_state.osd_heading;
  // The UAV-relative (0-360, 0 being front of UAV) bearing to home. 
  // This is the angle used for display of the direction-to-home arrow.
  float relative_home_bearing = 0.0f;
  
  // Bearing to home should consistently show 0 degrees (north) until 
  // home position is actually set.
  int got_home = get_osd_got_home();
  if (got_home == 1) {
      relative_home_bearing = absolute_home_bearing - uav_compass_bearing;
  }
  
  Reset_Polygon2D(&home_direction);
  Reset_Polygon2D(&home_direction_outline);
  Rotate_Polygon2D(&home_direction, relative_home_bearing);
  Rotate_Polygon2D(&home_direction_outline, relative_home_bearing);

  const int x = home_direction.x0;
  const int y = home_direction.y0;

  // Only fill in interior if home has been set
  if (got_home == 1) {
      for (int i = 0; i < home_direction.num_verts; i += 2) {
        write_line_lm(home_direction.vlist_trans[i].x + x,
                      home_direction.vlist_trans[i].y + y,
                      home_direction.vlist_trans[i + 1].x + x,
                      home_direction.vlist_trans[i + 1].y + y,
                      1, 1);
      }      
  }  

  // Always show the outline
  for (int i = 0; i < home_direction.num_verts; i += 2) {
    write_line_lm(home_direction_outline.vlist_trans[i].x + x,
                  home_direction_outline.vlist_trans[i].y + y,
                  home_direction_outline.vlist_trans[i + 1].x + x,
                  home_direction_outline.vlist_trans[i + 1].y + y,
                  1, 0);
  }

  // For debugging the infamous bad home direction bug
  draw_home_direction_debug_info(x, y, absolute_home_bearing, uav_compass_bearing, relative_home_bearing);
}

// Debug output for direction to home calculation
void draw_home_direction_debug_info(int x, int y, float absolute_home_bearing, float uav_compass_bearing, float relative_home_bearing)
{
  char tmp_str[15] = { 0 };
  // font_index: 0 = small, 1 = medium, 2 = large
  int font_index = 0;
  
  sprintf(tmp_str, "abs h.b. %d", (int32_t)absolute_home_bearing);
  write_string(tmp_str, x, y + 15, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[font_index]);
  
  sprintf(tmp_str, "rel h.b. %d", (int32_t)relative_home_bearing);
  write_string(tmp_str, x, y + 30, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[font_index]);
  
  sprintf(tmp_str, "comp. b %d", (int32_t)uav_compass_bearing);
  write_string(tmp_str, x, y + 45, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[font_index]);  
}

void draw_uav2d() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Atti_mp_en,
                              eeprom_buffer.params.Atti_mp_panel)) {
    return;
  }

  if (eeprom_buffer.params.Atti_mp_type == 0) {
    draw_radar();
  } else {
    draw_simple_attitude();
  }
}

void draw_throttle(void) {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Throt_en,
                             eeprom_buffer.params.Throt_panel)) {
    return;
  }

  char tmp_str[10] = { 0 };
  int16_t pos_th_y, pos_th_x;
  int posX, posY;
  posX = eeprom_buffer.params.Throt_posX;
  posY = eeprom_buffer.params.Throt_posY;

  pos_th_y = (int16_t)(0.5 * osdproc_osd_state.osd_throttle);
  pos_th_x = posX - 25 + pos_th_y;
  sprintf(tmp_str, "%d%%", (int32_t)osdproc_osd_state.osd_throttle);
  write_string(tmp_str, posX, posY, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[0]);

  if (eeprom_buffer.params.Throt_scale_en) {
    pos_th_y = (int16_t)(0.5 * osdproc_osd_state.osd_throttle);
    pos_th_x = posX - 25 + pos_th_y;
    sprintf(tmp_str, "%d%%", (int32_t)osdproc_osd_state.osd_throttle);
    write_string(tmp_str, posX, posY, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[0]);
    if (eeprom_buffer.params.Throttle_Scale_Type == 0) {
      write_filled_rectangle_lm(posX + 3, posY + 25 - pos_th_y, 5, pos_th_y, 1, 1);
      write_hline_lm(posX + 3, posX + 7, posY - 25, 1, 1);
      write_hline_lm(posX + 3, posX + 7, posY + 25 - pos_th_y, 1, 1);
      write_vline_lm(posX + 3, posY - 25, posY + 25 - pos_th_y, 1, 1);
      write_vline_lm(posX + 7, posY - 25, posY + 25 - pos_th_y, 1, 1);
    }
    else if (eeprom_buffer.params.Throttle_Scale_Type == 1) {
      write_filled_rectangle_lm(posX - 25, posY + 10, pos_th_y, 5, 1, 1);
      write_hline_lm(pos_th_x, posX + 25, posY + 10, 1, 1);
      write_hline_lm(pos_th_x, posX + 25, posY + 14, 1, 1);
      write_vline_lm(posX + 25, posY + 10, posY + 14, 1, 1);
      write_vline_lm(posX - 25, posY + 10, posY + 14, 1, 1);
    }
  } else {
    pos_th_y = (int16_t)(0.5 * osdproc_osd_state.osd_throttle);
    sprintf(tmp_str, "THR %d%%", (int32_t)osdproc_osd_state.osd_throttle);
    write_string(tmp_str, posX, posY, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[0]);
  }
}

void draw_home_latitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.HomeLatitude_enabled,
                              eeprom_buffer.params.HomeLatitude_panel)) {
    return;
  }

  float osd_home_lat = get_osd_home_lat();
  
  sprintf(tmp_str, "H %0.5f", (double) osd_home_lat / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.HomeLatitude_posX,
               eeprom_buffer.params.HomeLatitude_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.HomeLatitude_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.HomeLatitude_fontsize]);
}

void draw_home_longitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.HomeLongitude_enabled,
                              eeprom_buffer.params.HomeLongitude_panel)) {
    return;
  }

  float osd_home_lon = get_osd_home_lon();
  
  sprintf(tmp_str, "H %0.5f", (double) osd_home_lon / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.HomeLongitude_posX,
               eeprom_buffer.params.HomeLongitude_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.HomeLongitude_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.HomeLongitude_fontsize]);
}

void draw_gps_status() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.GpsStatus_en,
                              eeprom_buffer.params.GpsStatus_panel)) {
    return;
  }

  switch (osdproc_osd_state.osd_fix_type) {
  case NO_GPS:
    // Only show this message when in fact there is no GPS attached
    sprintf(tmp_str, "NOGPS");
    break;
  case NO_FIX:
    sprintf(tmp_str, "NOFIX");
    break;
  case GPS_OK_FIX_2D:
    sprintf(tmp_str, "2D-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;
  case GPS_OK_FIX_3D:
    sprintf(tmp_str, "3D-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;
  case GPS_OK_FIX_3D_DGPS:
    sprintf(tmp_str, "D3D-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;    
  // I don't expect users will ever see these in the real world, but let's
  // handle them anyway
  case GPS_OK_FIX_3D_RTK_FLOAT:
    sprintf(tmp_str, "RTK-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;    
  case GPS_OK_FIX_TYPE_RTK_FIXED:
    sprintf(tmp_str, "RTKF-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;    
  case GPS_OK_FIX_TYPE_STATIC:
    sprintf(tmp_str, "STAT-%d", (int) osdproc_osd_state.osd_satellites_visible);
    break;
  default:
    // Some unknown GPS status, do our best to show we don't understand it in the 
    // tiny space allowed, showing fix code and number of satellites locked
    sprintf(tmp_str, "?FT:%d-%d", (int) osdproc_osd_state.osd_fix_type, (int) osdproc_osd_state.osd_satellites_visible);
    break;
  }
  write_string(tmp_str, eeprom_buffer.params.GpsStatus_posX,
               eeprom_buffer.params.GpsStatus_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.GpsStatus_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.GpsStatus_fontsize]);
}

void draw_gps_hdop() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.GpsHDOP_en,
                              eeprom_buffer.params.GpsHDOP_panel)) {
    return;
  }

  sprintf(tmp_str, "HDOP %0.1f", (double) osdproc_osd_state.osd_hdop / 100.0f);
  write_string(tmp_str, eeprom_buffer.params.GpsHDOP_posX,
               eeprom_buffer.params.GpsHDOP_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.GpsHDOP_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.GpsHDOP_fontsize]);
}

void draw_gps_latitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.GpsLat_en,
                              eeprom_buffer.params.GpsLat_panel)) {
    return;
  }
  
  sprintf(tmp_str, "%0.5f", (double) osdproc_osd_state.osd_lat / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.GpsLat_posX,
               eeprom_buffer.params.GpsLat_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.GpsLat_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.GpsLat_fontsize]);
}

void draw_gps_longitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.GpsLon_en,
                              eeprom_buffer.params.GpsLon_panel)) {
    return;
  } 

  sprintf(tmp_str, "%0.5f", (double) osdproc_osd_state.osd_lon / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.GpsLon_posX,
               eeprom_buffer.params.GpsLon_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.GpsLon_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.GpsLon_fontsize]);
}

void draw_gps2_status() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Gps2Status_en,
                              eeprom_buffer.params.Gps2Status_panel)) {
    return;
  }

  switch (osdproc_osd_state.osd_fix_type2) {
  case NO_GPS:
  case NO_FIX:
    sprintf(tmp_str, "NOFIX");
    break;
  case GPS_OK_FIX_2D:
    sprintf(tmp_str, "2D-%d", (int) osdproc_osd_state.osd_satellites_visible2);
    break;
  case GPS_OK_FIX_3D:
    sprintf(tmp_str, "3D-%d", (int) osdproc_osd_state.osd_satellites_visible2);
    break;
  case GPS_OK_FIX_3D_DGPS:
    sprintf(tmp_str, "D3D-%d", (int) osdproc_osd_state.osd_satellites_visible2);
    break;
  default:
    sprintf(tmp_str, "NOGPS");
    break;
  }
  write_string(tmp_str, eeprom_buffer.params.Gps2Status_posX,
               eeprom_buffer.params.Gps2Status_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Gps2Status_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Gps2Status_fontsize]);
}

void draw_gps2_hdop() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Gps2HDOP_en,
                              eeprom_buffer.params.Gps2HDOP_panel)) {
    return;
  }

  sprintf(tmp_str, "HDOP %0.1f", (double) osdproc_osd_state.osd_hdop2 / 100.0f);
  write_string(tmp_str, eeprom_buffer.params.Gps2HDOP_posX,
               eeprom_buffer.params.Gps2HDOP_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Gps2HDOP_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Gps2HDOP_fontsize]);
}

void draw_gps2_latitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Gps2Lat_en,
                              eeprom_buffer.params.Gps2Lat_panel)) {
    return;
  }

  sprintf(tmp_str, "%0.5f", (double) osdproc_osd_state.osd_lat2 / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.Gps2Lat_posX,
               eeprom_buffer.params.Gps2Lat_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Gps2Lat_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Gps2Lat_fontsize]);
}

void draw_gps2_longitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Gps2Lon_en,
                              eeprom_buffer.params.Gps2Lon_panel)) {
    return;
  }

  sprintf(tmp_str, "%0.5f", (double) osdproc_osd_state.osd_lon2 / DEGREE_MULTIPLIER);
  write_string(tmp_str, eeprom_buffer.params.Gps2Lon_posX,
               eeprom_buffer.params.Gps2Lon_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Gps2Lon_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Gps2Lon_fontsize]);
}

void draw_total_trip() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.TotalTripDist_en,
                              eeprom_buffer.params.TotalTripDist_panel)) {
    return;
  }

  float tmp = 0.0f;
  // Get total trip distance using the ad-hoc mutex & global structure  
  if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
      tmp = adhoc_osd_state.osd_total_trip_dist * convert_distance; 
      // Release the ad-hoc mutex
      xSemaphoreGive(osd_state_adhoc_mutex);
  }  
  
  if (tmp < convert_distance_divider) {
    sprintf(tmp_str, "%d%s", (int) tmp, dist_unit_short);
  }
  else{
    sprintf(tmp_str, "%0.2f%s", (double) (tmp / convert_distance_divider), dist_unit_long);
  }
  write_string(tmp_str, eeprom_buffer.params.TotalTripDist_posX,
               eeprom_buffer.params.TotalTripDist_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.TotalTripDist_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.TotalTripDist_fontsize]);
}

void draw_time() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Time_en,
                              eeprom_buffer.params.Time_panel)) {
    return;
  }

  uint32_t time_now = GetSystimeMS() - get_sys_start_time();

  if (eeprom_buffer.params.Time_type == 1) {
    uint32_t heartbeat_start_time = get_heartbeat_start_time();
    time_now = heartbeat_start_time ? (GetSystimeMS() - heartbeat_start_time) : 0;
  } else if (eeprom_buffer.params.Time_type == 2) {
    uint32_t armed_start_time = get_armed_start_time();
    uint32_t total_armed_time = get_total_armed_time();
    time_now = armed_start_time ? (GetSystimeMS() - armed_start_time + total_armed_time) : total_armed_time;
  }

  int16_t tmp_int16 = (time_now / 3600000);     // hours
  int tmp_int1 = 0;
  int tmp_int2 = 0;
  if (tmp_int16 == 0) {
    tmp_int1 = time_now / 60000;       // minutes
    tmp_int2 = (time_now / 1000) - 60 * tmp_int1;       // seconds
    sprintf(tmp_str, "%02d:%02d", (int) tmp_int1, (int) tmp_int2);
  } else {
    tmp_int1 = time_now / 60000 - 60 * tmp_int16;       // minutes
    tmp_int2 = (time_now / 1000) - 60 * tmp_int1 - 3600 * tmp_int16;       // seconds
    sprintf(tmp_str, "%02d:%02d:%02d", (int) tmp_int16, (int) tmp_int1, (int) tmp_int2);
  }
  write_string(tmp_str, eeprom_buffer.params.Time_posX,
               eeprom_buffer.params.Time_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Time_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Time_fontsize]);
}

// Haversine distance
// http://www.movable-type.co.uk/scripts/latlong.html
float get_distance_between_locations_in_meters(float lat_from,
                                               float lon_from,
                                               float lat_to,
                                               float lon_to) {
    float R = 6371e3; // metres

    lat_from = lat_from / DEGREE_MULTIPLIER;
    lon_from = lon_from / DEGREE_MULTIPLIER;
    lat_to = lat_to / DEGREE_MULTIPLIER;
    lon_to = lon_to / DEGREE_MULTIPLIER;

    float phi_one = Convert_Angle_To_Radians(lat_from);
    float phi_two = Convert_Angle_To_Radians(lat_to);
    float delta_phi = Convert_Angle_To_Radians(lat_to - lat_from);
    float delta_lambda = Convert_Angle_To_Radians(lon_to - lon_from);

    // I suggest we avoid using optimized math functions until we have
    // something working well. -- SLG
    float a = sin(delta_phi/2) * sin(delta_phi/2) +
            cos(phi_one) * cos(phi_two) *
            sin(delta_lambda/2) * sin(delta_lambda/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));

    float distance = R * c;
    return distance;
}

float get_distance_from_home_in_meters() {    
    // Distance to home is always 0 if home isn't set yet
    if (get_osd_got_home() == 0) {
        return 0.0f;
    }
        
    float osd_lat_current  = osdproc_osd_state.osd_lat;
    float osd_lon_current  = osdproc_osd_state.osd_lon;
    float osd_home_lat = get_osd_home_lat();
    float osd_home_lon = get_osd_home_lon();
      
    return get_distance_between_locations_in_meters(osd_home_lat, osd_home_lon, osd_lat_current, osd_lon_current);
}

// Thanks again to:
// http://www.movable-type.co.uk/scripts/latlong.html
float get_absolute_bearing_to_home_in_degrees() {   

    // Bearing to home is always 0 (north) if home isn't set yet
    if (get_osd_got_home() == 0) {
        return 0.0f;
    }

    float osd_lat_current  = osdproc_osd_state.osd_lat;
    float osd_lon_current  = osdproc_osd_state.osd_lon; 
    float osd_home_lat = get_osd_home_lat();
    float osd_home_lon = get_osd_home_lon();    
      
    float phi_1 = Convert_Angle_To_Radians(osd_lat_current / DEGREE_MULTIPLIER);
    float phi_2 = Convert_Angle_To_Radians(osd_home_lat / DEGREE_MULTIPLIER);
    float delta_lambda = Convert_Angle_To_Radians((osd_home_lon / DEGREE_MULTIPLIER) - (osd_lon_current / DEGREE_MULTIPLIER));

    // see http://mathforum.org/library/drmath/view/55417.html
    float y = sin(delta_lambda) * cos(phi_2);
    float x = cos(phi_1) * sin(phi_2) -
              sin(phi_1) * cos(phi_2) * cos(delta_lambda);
    float theta = atan2(y, x);
    float final_angle = fmod((Convert_Radians_To_Angle(theta)+360.0f), 360.0f);
    return final_angle;
}

void draw_distance_to_home() {
    if (!enabledAndShownOnPanel(eeprom_buffer.params.CWH_home_dist_en, 
                              eeprom_buffer.params.CWH_home_dist_panel)) {
      return;
    }
        
    float tmp = get_osd_home_distance() * convert_distance;
    
    // If home not set, give some indication that distance to home is currently meaningless
    if (get_osd_got_home() == 0){
        sprintf(tmp_str, "H -%s", dist_unit_short);
    // Display short units (meters/feet)
    } else if (tmp < convert_distance_divider) {
        sprintf(tmp_str, "H %d%s", (int)tmp, dist_unit_short);
    // Display long units (kilometers/miles)
    } else {
        sprintf(tmp_str, "H %0.2f%s", (double)(tmp / convert_distance_divider), dist_unit_long);
    }

    write_string(tmp_str, eeprom_buffer.params.CWH_home_dist_posX, eeprom_buffer.params.CWH_home_dist_posY, 0, 0, TEXT_VA_TOP, eeprom_buffer.params.CWH_home_dist_align, 0, SIZE_TO_FONT[eeprom_buffer.params.CWH_home_dist_fontsize]);  
}

void draw_distance_to_waypoint() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.CWH_wp_dist_en, 
                              eeprom_buffer.params.CWH_wp_dist_panel)) {
      return;
  } 
    
    if (osdproc_osd_state.wp_number != 0) {
         float tmp = osdproc_osd_state.wp_dist * convert_distance;
         if (tmp < convert_distance_divider) {
            sprintf(tmp_str, "WP %d%s", (int)tmp, dist_unit_short);
         }
         else {
            sprintf(tmp_str, "WP %0.2f%s", (double)(tmp / convert_distance_divider), dist_unit_long);
         }

        write_string(tmp_str, eeprom_buffer.params.CWH_wp_dist_posX, eeprom_buffer.params.CWH_wp_dist_posY, 0, 0, TEXT_VA_TOP, eeprom_buffer.params.CWH_wp_dist_align, 0, SIZE_TO_FONT[eeprom_buffer.params.             CWH_wp_dist_fontsize]);
    }
}

// Set Home Position if needed.
void set_home_position_if_unset() {
  if ((get_osd_got_home() == 0) && (osdproc_osd_state.motor_armed) && (osdproc_osd_state.osd_fix_type > 1)) {
      
    float osd_lat_current  = osdproc_osd_state.osd_lat;
    float osd_lon_current  = osdproc_osd_state.osd_lon;
    
    set_osd_home_lat(osd_lat_current);
    set_osd_home_lon(osd_lon_current);
    get_osd_alt_cnt(0);
    set_osd_got_home(1);
  }
}

// Set home altitude if needed
void set_home_altitude_if_unset() {
    if (get_osd_got_home() == 1)
    {
        float osd_alt_current = osdproc_osd_state.osd_alt;
        uint8_t osd_alt_cnt_current = get_osd_alt_cnt();
        float osd_alt_prev = get_osd_alt_prev();
        
        // JRChange: osd_home_alt: check for stable osd_alt (must be stable for 75*40ms = 3s)
        // we can get the relative alt from mavlink directly.
        if (osd_alt_cnt_current < 75) {
          if (fabs(osd_alt_prev - osd_alt_current) > 0.5) {
            set_osd_alt_cnt(0);
            set_osd_alt_prev(osd_alt_current);
          } else {
            set_osd_alt_cnt(osd_alt_cnt_current++);
            if (osd_alt_cnt_current >= 75) {
              set_osd_home_alt(osd_alt_current);           // take this stable osd_alt as osd_home_alt
            }
          }
        }
    }
}

// Set home distance and bearing, if we know where home is
void set_home_distance_and_bearing() {
  if (get_osd_got_home() == 1) {
    set_osd_home_distance(get_distance_from_home_in_meters());
    set_osd_home_bearing(get_absolute_bearing_to_home_in_degrees());
  }
}

// direction - scale mode
void draw_osd_linear_compass() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.CWH_Tmode_en, 
                              eeprom_buffer.params.CWH_Tmode_panel)) {
      return;
  }
  
  draw_linear_compass(osdproc_osd_state.osd_heading, 0, 120, 180, GRAPHICS_X_MIDDLE, eeprom_buffer.params.CWH_Tmode_posY, 15, 30, 5, 8, 0);
}

float get_updated_average_climb_rate() {
    float average_climb = 0.0f;
    
    // Take the ad-hoc global mutex
    if (xSemaphoreTake(osd_state_adhoc_mutex, portMAX_DELAY) == pdTRUE ) {
        adhoc_osd_state.osd_climb_ma[adhoc_osd_state.osd_climb_ma_index] = osdproc_osd_state.osd_climb;
        adhoc_osd_state.osd_climb_ma_index = (adhoc_osd_state.osd_climb_ma_index + 1) % 10;
        for (int i = 0; i < 10; i++) {
            average_climb = average_climb + adhoc_osd_state.osd_climb_ma[i];
        }
        average_climb = roundf(10 * (average_climb / 10)) / 10.0f;

        // Release the ad-hoc mutex
        xSemaphoreGive(osd_state_adhoc_mutex);
    }
    return average_climb;
}


void draw_climb_rate() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.ClimbRate_en,
                              eeprom_buffer.params.ClimbRate_panel)) {
    return;
  }
  
  float average_climb = get_updated_average_climb_rate();
  int x = eeprom_buffer.params.ClimbRate_posX;
  int y = eeprom_buffer.params.ClimbRate_posY;
  sprintf(tmp_str, "%0.1f", fabs(average_climb));
  write_string(tmp_str, x + 5, y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0,
               SIZE_TO_FONT[eeprom_buffer.params.ClimbRate_fontsize]);

  int arrowLength = 6;
  if (eeprom_buffer.params.ClimbRate_fontsize != 0) {
    arrowLength += 2;
  }

  if (average_climb > 0.0f) {
    write_vline_lm(x, y - arrowLength, y + arrowLength, 1, 1);
    write_line_outlined(x - 3, y - arrowLength + 3, x, y - arrowLength, 2, 2, 0, 1);
    write_line_outlined(x + 3, y - arrowLength + 3, x, y - arrowLength, 2, 2, 0, 1);
  } else if (average_climb < 0.0f) {
    write_vline_lm(x, y - arrowLength, y + arrowLength, 1, 1);
    write_line_outlined(x - 3, y + arrowLength - 3, x, y + arrowLength, 2, 2, 0, 1);
    write_line_outlined(x + 3, y + arrowLength - 3, x, y + arrowLength, 2, 2, 0, 1);
  }
}

void draw_rssi() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.RSSI_en,
                              eeprom_buffer.params.RSSI_panel)) {
    return;
  }

  int rssi = (int)osdproc_osd_state.osd_rssi;

  //Not from the MAVLINK, should take the RC channel PWM value.
  if (eeprom_buffer.params.RSSI_type != 0)
  {
    if (eeprom_buffer.params.RSSI_type == 5) rssi = (int)osdproc_osd_state.osd_chan5_raw;
    else if (eeprom_buffer.params.RSSI_type == 6) rssi = (int)osdproc_osd_state.osd_chan6_raw;
    else if (eeprom_buffer.params.RSSI_type == 7) rssi = (int)osdproc_osd_state.osd_chan7_raw;
    else if (eeprom_buffer.params.RSSI_type == 8) rssi = (int)osdproc_osd_state.osd_chan8_raw;
    else if (eeprom_buffer.params.RSSI_type == 9) rssi = (int)osdproc_osd_state.osd_chan9_raw;
    else if (eeprom_buffer.params.RSSI_type == 10) rssi = (int)osdproc_osd_state.osd_chan10_raw;
    else if (eeprom_buffer.params.RSSI_type == 11) rssi = (int)osdproc_osd_state.osd_chan11_raw;
    else if (eeprom_buffer.params.RSSI_type == 12) rssi = (int)osdproc_osd_state.osd_chan12_raw;
    else if (eeprom_buffer.params.RSSI_type == 13) rssi = (int)osdproc_osd_state.osd_chan13_raw;
    else if (eeprom_buffer.params.RSSI_type == 14) rssi = (int)osdproc_osd_state.osd_chan14_raw;
    else if (eeprom_buffer.params.RSSI_type == 15) rssi = (int)osdproc_osd_state.osd_chan15_raw;
    else if (eeprom_buffer.params.RSSI_type == 16) rssi = (int)osdproc_osd_state.osd_chan16_raw;
  }

  //0:percentage 1:raw
  if ((eeprom_buffer.params.RSSI_raw_en == 0)) {
    uint16_t rssiMin = eeprom_buffer.params.RSSI_min;
    uint16_t rssiMax = eeprom_buffer.params.RSSI_max;

    //Trim the rssiMAX/MIN only for MAVLINK-rssi
    if (eeprom_buffer.params.RSSI_type == 0) {
      if (rssiMin < 0)
        rssiMin = 0;
      if (rssiMax > 255)
        rssiMax = 255;
    }

    if ((rssiMax - rssiMin) > 0)
      rssi = (int) ((float) (rssi - rssiMin) / (float) (rssiMax - rssiMin) * 100.0f);

    if (rssi < 0)
      rssi = 0;
    sprintf(tmp_str, "RSSI %d%%", rssi);
  } else {
    sprintf(tmp_str, "RSSI %d", rssi);
  }

  write_string(tmp_str, eeprom_buffer.params.RSSI_posX,
               eeprom_buffer.params.RSSI_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.RSSI_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.RSSI_fontsize]);
}

void draw_link_quality() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.LinkQuality_en,
                              eeprom_buffer.params.LinkQuality_panel)) {
    return;
  }

  int linkquality = (int)linkquality;
  int min = eeprom_buffer.params.LinkQuality_min;
  int max = eeprom_buffer.params.LinkQuality_max;

  if (eeprom_buffer.params.LinkQuality_chan == 5) linkquality = (int)osdproc_osd_state.osd_chan5_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 6) linkquality = (int)osdproc_osd_state.osd_chan6_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 7) linkquality = (int)osdproc_osd_state.osd_chan7_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 8) linkquality = (int)osdproc_osd_state.osd_chan8_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 9) linkquality = (int)osdproc_osd_state.osd_chan9_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 10) linkquality = (int)osdproc_osd_state.osd_chan10_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 11) linkquality = (int)osdproc_osd_state.osd_chan11_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 12) linkquality = (int)osdproc_osd_state.osd_chan12_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 13) linkquality = (int)osdproc_osd_state.osd_chan13_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 14) linkquality = (int)osdproc_osd_state.osd_chan14_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 15) linkquality = (int)osdproc_osd_state.osd_chan15_raw;
  else if (eeprom_buffer.params.LinkQuality_chan == 16) linkquality = (int)osdproc_osd_state.osd_chan16_raw;

  // 0: percent, 1: raw
  if (eeprom_buffer.params.LinkQuality_type == 0) {
    //OpenLRS will output 0 instead of min if the RX is powerd up before the TX
    if (linkquality < min)
    {
      linkquality = min;
    }

    //Get rid of any variances
    if (linkquality > max)
    {
      linkquality = max;
    }

    //Funky Conversion from  pwm min & max to percent
    linkquality = (int) ((float) (linkquality - max) / (float) (max - min) * 100.0f) + 100;
    sprintf(tmp_str, "LIQU %d%%", linkquality);
  } else {
    sprintf(tmp_str, "LIQU %d", linkquality);
  }

  write_string(tmp_str, eeprom_buffer.params.LinkQuality_posX,
               eeprom_buffer.params.LinkQuality_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.LinkQuality_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.LinkQuality_fontsize]);
}

void draw_efficiency() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Efficiency_en,
                              eeprom_buffer.params.Efficiency_panel)) {
    return;
  }

  float wattage = osdproc_osd_state.osd_vbat_A * osdproc_osd_state.osd_curr_A * 0.01;
  float speed = osdproc_osd_state.osd_groundspeed * convert_speed;
  float efficiency = 0;
  if (speed != 0) {
    efficiency = wattage / speed;
  }
  sprintf(tmp_str, "%0.1fW/%s", efficiency, dist_unit_long);

  write_string(tmp_str, eeprom_buffer.params.Efficiency_posX, eeprom_buffer.params.Efficiency_posY,
               0, 0, TEXT_VA_TOP, eeprom_buffer.params.Efficiency_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Efficiency_fontsize]);
}

void draw_watts() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Watts_en,
                              eeprom_buffer.params.Watts_panel)) {
    return;
  }

  sprintf(tmp_str, "%0.1fW", osdproc_osd_state.osd_vbat_A * osdproc_osd_state.osd_curr_A * 0.01);

  write_string(tmp_str, eeprom_buffer.params.Watts_posX, eeprom_buffer.params.Watts_posY,
               0, 0, TEXT_VA_TOP, eeprom_buffer.params.Watts_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Watts_fontsize]);
}

void draw_panel_changed() {
  uint8_t current_panel = get_current_panel();
  if (last_panel != current_panel) {
    last_panel = current_panel;
    new_panel_start_time = GetSystimeMS();
  }

  if ((GetSystimeMS() - new_panel_start_time) < 3000) {
    sprintf(tmp_str, "P %d", (int) current_panel);
    write_string(tmp_str, GRAPHICS_X_MIDDLE, 210, 0, 0, TEXT_VA_TOP,
                 TEXT_HA_CENTER, 0, SIZE_TO_FONT[1]);
  }
}

// Version splash screen that appears at startup
void draw_version_splash() {  

  // We're done showing it
  if (version_splash_shown == true) {
      return;
  }
    // If time to show is 0, don't show at all  
    if (eeprom_buffer.params.version_splash_milliseconds_to_show == 0){
      return;
  }
  
  // If we've already shown the splash long enough, no need to show any more
  uint32_t milliseconds_elapsed_since_boot = (GetSystimeMS() - version_splash_start_time);
  if (milliseconds_elapsed_since_boot > eeprom_buffer.params.version_splash_milliseconds_to_show) {
      // We've shown it long enough, mark it shown
      version_splash_shown = true;
      return;          
  }

  // Take note of the first time we start showing the splash screen
  if (version_splash_start_time == 0) {
    version_splash_start_time = GetSystimeMS();
  }
        
  // Show the splash screen
  int font_number_line_one = 0;
  struct FontEntry font_info_line_one;
  fetch_font_info(0, font_number_line_one, &font_info_line_one, NULL);
  char version_str_line_one[15];
  struct FontDimensions line_one_font_dim;
  memset (version_str_line_one, ' ', 15);    
  sprintf(version_str_line_one, "PLAYUAV V%s", PLAYUAV_VERSION_NUMBER);
  calc_text_dimensions(version_str_line_one, font_info_line_one, 1, 0, &line_one_font_dim);
  
  int font_number_line_two = 1;
  struct FontEntry font_info_line_two;
  fetch_font_info(0, font_number_line_two, &font_info_line_two, NULL);
  char version_str_line_two[15];
  struct FontDimensions line_two_font_dim;
  memset (version_str_line_two, ' ', 15);
  sprintf(version_str_line_two, "%s", PLAYUAV_VERSION_DESCRIPTION);
  calc_text_dimensions(version_str_line_two, font_info_line_two, 1, 0, &line_two_font_dim);    
  
  int horizontal_padding = 20;
  int vertical_padding = 20;
  int splash_box_width = MAX(line_one_font_dim.width, line_two_font_dim.width) + (horizontal_padding * 2);
  int splash_box_height = line_one_font_dim.height + line_two_font_dim.height + (vertical_padding * 2);
  int splash_rect_x = GRAPHICS_X_MIDDLE - (splash_box_width / 2);
  int splash_rect_y = GRAPHICS_Y_MIDDLE - (splash_box_height / 2);
  int text_pos_x = GRAPHICS_X_MIDDLE; 
  int text_pos_y = GRAPHICS_Y_MIDDLE; 
  
  write_filled_rectangle_lm(splash_rect_x, splash_rect_y, splash_box_width, splash_box_height, 1, 0);
  write_rectangle_outlined(splash_rect_x, splash_rect_y, splash_box_width, splash_box_height, 0, 1);
  write_string(version_str_line_one, text_pos_x, text_pos_y, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, font_number_line_one);
  write_string(version_str_line_two, text_pos_x, text_pos_y + line_one_font_dim.height, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, font_number_line_two);
}


/**
 * hud_draw_compass: Draw a compass.
 *
 * @param       v               value for the compass
 * @param       range           range about value to display (+/- range/2 each direction)
 * @param       width           length in pixels
 * @param       x               x displacement
 * @param       y               y displacement
 * @param       mintick_step    how often a minor tick is shown
 * @param       majtick_step    how often a major tick (heading "xx") is shown
 * @param       mintick_len     minor tick length
 * @param       majtick_len     major tick length
 * @param       flags           special flags (see hud.h.)
 */
#define COMPASS_SMALL_NUMBER
// #define COMPASS_FILLED_NUMBER
void draw_linear_compass(int v, int home_dir, int range, int width, int x, int y, int mintick_step, int majtick_step, int mintick_len, int majtick_len, __attribute__((unused)) int flags) {
  v %= 360;   // wrap, just in case.
  struct FontEntry font_info;
  int majtick_start = 0, majtick_end = 0, mintick_start = 0, mintick_end = 0, textoffset = 0;
  char headingstr[4];
  majtick_start = y;
  majtick_end   = y - majtick_len;
  mintick_start = y;
  mintick_end   = y - mintick_len;
  textoffset    = 8;
  int r, style, rr, xs;   // rv,
  int range_2 = range / 2;
  bool home_drawn = false;

//  home_dir = 30;
//  int wp_dir = 60;

  for (r = -range_2; r <= +range_2; r++) {
    style = 0;
    rr    = (v + r + 360) % 360;     // normalise range for modulo, add to move compass track
    // rv = -rr + range_2; // for number display
    if (rr % majtick_step == 0) {
      style = 1;       // major tick
    } else if (rr % mintick_step == 0) {
      style = 2;       // minor tick
    }
    if (style) {
      // Calculate x position.
      xs = ((long int)(r * width) / (long int)range) + x;
      // Draw it.
      if (style == 1) {
        write_vline_outlined(xs, majtick_start, majtick_end, 2, 2, 0, 1);
        // Draw heading above this tick.
        // If it's not one of north, south, east, west, draw the heading.
        // Otherwise, draw one of the identifiers.
        if (rr % 90 != 0) {
          // We abbreviate heading to two digits. This has the side effect of being easy to compute.
          headingstr[0] = '0' + (rr / 100);
          headingstr[1] = '0' + ((rr / 10) % 10);
          headingstr[2] = 0;
          headingstr[3] = 0;           // nul to terminate
        } else {
          switch (rr) {
          case 0:
            headingstr[0] = 'N';
            break;
          case 90:
            headingstr[0] = 'E';
            break;
          case 180:
            headingstr[0] = 'S';
            break;
          case 270:
            headingstr[0] = 'W';
            break;
          }
          headingstr[1] = 0;
          headingstr[2] = 0;
          headingstr[3] = 0;
        }
        // +1 fudge...!
        write_string(headingstr, xs + 1, majtick_start + textoffset, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 1);

      } else if (style == 2) {
        write_vline_outlined(xs, mintick_start, mintick_end, 2, 2, 0, 1);
      }
    }

//      // Put home direction
//      if (rr == home_dir) {
//          xs = ((long int)(r * width) / (long int)range) + x;
//          write_filled_rectangle_lm(xs - 5, majtick_start + textoffset + 7, 10, 10, 0, 1);
//          write_string("H", xs + 1, majtick_start + textoffset + 12, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 1);
//          home_drawn = true;
//      }
//
//      // Put home direction
//      if (rr == wp_dir) {
//          xs = ((long int)(r * width) / (long int)range) + x;
//          write_filled_rectangle_lm(xs - 5, majtick_start + textoffset + 7, 10, 10, 0, 1);
//          write_string("W", xs + 1, majtick_start + textoffset + 12, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 1);
////            home_drawn = true;
//      }
  }

//  if (home_dir > 0 && !home_drawn) {
//      if (((v > home_dir) && (v - home_dir < 180)) || ((v < home_dir) && (home_dir -v > 180)))
//          r = x - ((long int)(range_2 * width) / (long int)range);
//      else
//          r = x + ((long int)(range_2 * width) / (long int)range);

//      write_filled_rectangle_lm(r - 5, majtick_start + textoffset + 7, 10, 10, 0, 1);
//      write_string("H", r + 1, majtick_start + textoffset + 12, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, 1);
//  }


  // Then, draw a rectangle with the present heading in it.
  // We want to cover up any other markers on the bottom.
  // First compute font size.
  headingstr[0] = '0' + (v / 100);
  headingstr[1] = '0' + ((v / 10) % 10);
  headingstr[2] = '0' + (v % 10);
  headingstr[3] = 0;
  fetch_font_info(0, 3, &font_info, NULL);
#ifdef COMPASS_SMALL_NUMBER
  int rect_width = font_info.width * 3;
#ifdef COMPASS_FILLED_NUMBER
  write_filled_rectangle_lm(x - (rect_width / 2), majtick_start - 7, rect_width, font_info.height, 0, 1);
#else
  write_filled_rectangle_lm(x - (rect_width / 2), majtick_start - 7, rect_width, font_info.height, 0, 0);
#endif
  write_rectangle_outlined(x - (rect_width / 2), majtick_start - 7, rect_width, font_info.height, 0, 1);
  write_string(headingstr, x + 1, majtick_start + textoffset - 5, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 1, 0);
#else
  int rect_width = (font_info.width + 1) * 3 + 2;
#ifdef COMPASS_FILLED_NUMBER
  write_filled_rectangle_lm(x - (rect_width / 2), majtick_start + 2, rect_width, font_info.height + 2, 0, 1);
#else
  write_filled_rectangle_lm(x - (rect_width / 2), majtick_start + 2, rect_width, font_info.height + 2, 0, 0);
#endif
  write_rectangle_outlined(x - (rect_width / 2), majtick_start + 2, rect_width, font_info.height + 2, 0, 1);
  write_string(headingstr, x + 1, majtick_start + textoffset + 2, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 1, 3);
#endif
}

/**
 * draw_vertical_scale: Draw a vertical scale.
 *
 * @param       v                   value to display as an integer
 * @param       range               range about value to display (+/- range/2 each direction)
 * @param       halign              horizontal alignment: 0 = left, 1 = right.
 * @param       x                   x displacement
 * @param       y                   y displacement
 * @param       height              height of scale
 * @param       mintick_step        how often a minor tick is shown
 * @param       majtick_step        how often a major tick is shown
 * @param       mintick_len         minor tick length
 * @param       majtick_len         major tick length
 * @param       boundtick_len       boundary tick length
 * @param       max_val             maximum expected value (used to compute size of arrow ticker)
 * @param       flags               special flags (see hud.h.)
 */
// #define VERTICAL_SCALE_BRUTE_FORCE_BLANK_OUT
#define VERTICAL_SCALE_FILLED_NUMBER
void draw_vertical_scale(int v, int range, int halign, int x, int y,
                         int height, int mintick_step, int majtick_step, int mintick_len,
                         int majtick_len, int boundtick_len, __attribute__((unused)) int max_val,
                         int flags) {
  char temp[15];
  struct FontEntry font_info;
  struct FontDimensions dim;
  // Compute the position of the elements.
  int majtick_start = 0, majtick_end = 0, mintick_start = 0, mintick_end = 0, boundtick_start = 0, boundtick_end = 0;

  majtick_start   = x;
  mintick_start   = x;
  boundtick_start = x;
  if (halign == 0) {
    majtick_end     = x + majtick_len;
    mintick_end     = x + mintick_len;
    boundtick_end   = x + boundtick_len;
  } else if (halign == 1) {
    majtick_end     = x - majtick_len;
    mintick_end     = x - mintick_len;
    boundtick_end   = x - boundtick_len;
  }
  // Retrieve width of large font (font #0); from this calculate the x spacing.
  fetch_font_info(0, 0, &font_info, NULL);
  int arrow_len      = (font_info.height / 2) + 1;
  int text_x_spacing = (font_info.width / 2);
  int max_text_y     = 0, text_length = 0;
  int small_font_char_width = font_info.width + 1;   // +1 for horizontal spacing = 1
  // For -(range / 2) to +(range / 2), draw the scale.
  int range_2 = range / 2;   // , height_2 = height / 2;
  int r = 0, rr = 0, rv = 0, ys = 0, style = 0;   // calc_ys = 0,
  // Iterate through each step.
  for (r = -range_2; r <= +range_2; r++) {
    style = 0;
    rr    = r + range_2 - v;     // normalise range for modulo, subtract value to move ticker tape
    rv    = -rr + range_2;     // for number display
    if (flags & HUD_VSCALE_FLAG_NO_NEGATIVE) {
      rr += majtick_step / 2;
    }
    if (rr % majtick_step == 0) {
      style = 1;       // major tick
    } else if (rr % mintick_step == 0) {
      style = 2;       // minor tick
    } else {
      style = 0;
    }
    if (flags & HUD_VSCALE_FLAG_NO_NEGATIVE && rv < 0) {
      continue;
    }
    if (style) {
      // Calculate y position.
      ys = ((long int)(r * height) / (long int)range) + y;
      // Depending on style, draw a minor or a major tick.
      if (style == 1) {
        write_hline_outlined(majtick_start, majtick_end, ys, 2, 2, 0, 1);
        memset(temp, ' ', 10);
        sprintf(temp, "%d", rv);
        text_length = (strlen(temp) + 1) * small_font_char_width;         // add 1 for margin
        if (text_length > max_text_y) {
          max_text_y = text_length;
        }
        if (halign == 0) {
          write_string(temp, majtick_end + text_x_spacing + 1, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0, 1);
        } else {
          write_string(temp, majtick_end - text_x_spacing + 1, ys, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_RIGHT, 0, 1);
        }
      } else if (style == 2) {
        write_hline_outlined(mintick_start, mintick_end, ys, 2, 2, 0, 1);
      }
    }
  }
  // Generate the string for the value, as well as calculating its dimensions.
  memset(temp, ' ', 10);
  // my_itoa(v, temp);
  sprintf(temp, "%d", v);
  // TODO: add auto-sizing.
  calc_text_dimensions(temp, font_info, 1, 0, &dim);
  int xx = 0, i = 0;
  if (halign == 0) {
    xx = majtick_end + text_x_spacing;
  } else {
    xx = majtick_end - text_x_spacing;
  }
  y++;
  // Draw an arrow from the number to the point.
  for (i = 0; i < arrow_len; i++) {
    if (halign == 0) {
      write_pixel_lm(xx - arrow_len + i, y - i - 1, 1, 1);
      write_pixel_lm(xx - arrow_len + i, y + i - 1, 1, 1);
#ifdef VERTICAL_SCALE_FILLED_NUMBER
      write_hline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y - i - 1, 0, 1);
      write_hline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y + i - 1, 0, 1);
#else
      write_hline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y - i - 1, 0, 0);
      write_hline_lm(xx + dim.width - 1, xx - arrow_len + i + 1, y + i - 1, 0, 0);
#endif
    } else {
      write_pixel_lm(xx + arrow_len - i, y - i - 1, 1, 1);
      write_pixel_lm(xx + arrow_len - i, y + i - 1, 1, 1);
#ifdef VERTICAL_SCALE_FILLED_NUMBER
      write_hline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y - i - 1, 0, 1);
      write_hline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y + i - 1, 0, 1);
#else
      write_hline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y - i - 1, 0, 0);
      write_hline_lm(xx - dim.width - 1, xx + arrow_len - i - 1, y + i - 1, 0, 0);
#endif
    }
  }
  if (halign == 0) {
    write_hline_lm(xx, xx + dim.width - 1, y - arrow_len, 1, 1);
    write_hline_lm(xx, xx + dim.width - 1, y + arrow_len - 2, 1, 1);
    write_vline_lm(xx + dim.width - 1, y - arrow_len, y + arrow_len - 2, 1, 1);
  } else {
    write_hline_lm(xx, xx - dim.width - 1, y - arrow_len, 1, 1);
    write_hline_lm(xx, xx - dim.width - 1, y + arrow_len - 2, 1, 1);
    write_vline_lm(xx - dim.width - 1, y - arrow_len, y + arrow_len - 2, 1, 1);
  }
  // Draw the text.
  if (halign == 0) {
    write_string(temp, xx, y, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0, 0);
  } else {
    write_string(temp, xx, y, 1, 0, TEXT_VA_MIDDLE, TEXT_HA_RIGHT, 0, 0);
  }
#ifdef VERTICAL_SCALE_BRUTE_FORCE_BLANK_OUT
  // This is a bad brute force method destuctive to other things that maybe drawn underneath like e.g. the artificial horizon:
  // Then, add a slow cut off on the edges, so the text doesn't sharply
  // disappear. We simply clear the areas above and below the ticker, and we
  // use little markers on the edges.
  if (halign == 0) {
    write_filled_rectangle_lm(majtick_end + text_x_spacing, y + (height / 2) - (font_info.height / 2), max_text_y - boundtick_start, font_info.height, 0, 0);
    write_filled_rectangle_lm(majtick_end + text_x_spacing, y - (height / 2) - (font_info.height / 2), max_text_y - boundtick_start, font_info.height, 0, 0);
  } else {
    write_filled_rectangle_lm(majtick_end - text_x_spacing - max_text_y, y + (height / 2) - (font_info.height / 2), max_text_y, font_info.height, 0, 0);
    write_filled_rectangle_lm(majtick_end - text_x_spacing - max_text_y, y - (height / 2) - (font_info.height / 2), max_text_y, font_info.height, 0, 0);
  }
#endif
  y--;
  write_hline_outlined(boundtick_start, boundtick_end, y + (height / 2), 2, 2, 0, 1);
  write_hline_outlined(boundtick_start, boundtick_end, y - (height / 2), 2, 2, 0, 1);
}

// direction - map-like mode
void draw_head_wp_home() {

  if (!enabledAndShownOnPanel(eeprom_buffer.params.CWH_Nmode_en,
                              eeprom_buffer.params.CWH_Nmode_panel)) {
    return;
  }

  int posX, posY, r;
  char tmp_str[10] = { 0 };

  //draw compass
  posX = eeprom_buffer.params.CWH_Nmode_posX;
  posY = eeprom_buffer.params.CWH_Nmode_posY;
  r = eeprom_buffer.params.CWH_Nmode_radius;
  write_circle_outlined(posX, posY, r, 0, 1, 0, 1);
//    write_string("N", posX, posY - r, 0, 0, TEXT_VA_TOP, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);

  //draw heading
  POLYGON2D suav;
  suav.state       = 1;
  suav.num_verts   = 3;
  suav.x0          = posX;
  suav.y0          = posY;
  VECTOR2D_INITXYZ(&(suav.vlist_local[0]), 0, -7);
  VECTOR2D_INITXYZ(&(suav.vlist_local[1]), -3, 7);
  VECTOR2D_INITXYZ(&(suav.vlist_local[2]), 3, 7);
  Reset_Polygon2D(&suav);
  Rotate_Polygon2D(&suav, osdproc_osd_state.osd_heading);
  write_line_outlined(suav.vlist_trans[0].x + suav.x0, suav.vlist_trans[0].y + suav.y0,
                      suav.vlist_trans[1].x + suav.x0, suav.vlist_trans[1].y + suav.y0, 2, 2, 0, 1);
  write_line_outlined(suav.vlist_trans[0].x + suav.x0, suav.vlist_trans[0].y + suav.y0,
                      suav.vlist_trans[2].x + suav.x0, suav.vlist_trans[2].y + suav.y0, 2, 2, 0, 1);
   
  uint32_t osd_home_bearing = get_osd_home_bearing();
  uint8_t home_is_set = get_osd_got_home();
  long home_distance = get_osd_home_distance();
  
  // Draw home 
  // Home only shown when home is set, and the distance to home is greater than 1 meter
  if (((int32_t)home_distance > 1) && (home_is_set == 1))
  {
    float homeCX = posX + (eeprom_buffer.params.CWH_Nmode_home_radius) * Fast_Sin(osd_home_bearing);
    float homeCY = posY - (eeprom_buffer.params.CWH_Nmode_home_radius) * Fast_Cos(osd_home_bearing);
    write_string("H", homeCX, homeCY, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);
  }

  //draw waypoint
  if ((osdproc_osd_state.wp_number != 0) && (osdproc_osd_state.wp_dist > 1))
  {
    //format bearing
    osdproc_osd_state.wp_target_bearing = (osdproc_osd_state.wp_target_bearing + 360) % 360;
    float wpCX = posX + (eeprom_buffer.params.CWH_Nmode_wp_radius) * Fast_Sin(osdproc_osd_state.wp_target_bearing);
    float wpCY = posY - (eeprom_buffer.params.CWH_Nmode_wp_radius) * Fast_Cos(osdproc_osd_state.wp_target_bearing);
    sprintf(tmp_str, "%d", (int)osdproc_osd_state.wp_number);
    write_string(tmp_str, wpCX, wpCY, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, 0, SIZE_TO_FONT[0]);
  }
}

// Show the current values of the RC Channels
void draw_rc_channels(void) {

    if (!enabledAndShownOnPanel(eeprom_buffer.params.RC_Channels_en,
                                eeprom_buffer.params.RC_Channels_panel)) {
        return;
    }
  
  char tmp_str[15] = { 0 };
  
  // Load channel values into an easier-to-use array.
  // Note the deliberate off-by-one addressing!
  uint16_t channel_values[17];
  
  channel_values[1] = osdproc_osd_state.osd_chan1_raw;
  channel_values[2] = osdproc_osd_state.osd_chan2_raw;
  channel_values[3] = osdproc_osd_state.osd_chan3_raw;
  channel_values[4] = osdproc_osd_state.osd_chan4_raw;
  channel_values[5] = osdproc_osd_state.osd_chan5_raw;
  channel_values[6] = osdproc_osd_state.osd_chan6_raw;
  channel_values[7] = osdproc_osd_state.osd_chan7_raw;
  channel_values[8] = osdproc_osd_state.osd_chan8_raw;
  channel_values[9] = osdproc_osd_state.osd_chan9_raw;
  channel_values[10] = osdproc_osd_state.osd_chan10_raw;
  channel_values[11] = osdproc_osd_state.osd_chan11_raw;
  channel_values[12] = osdproc_osd_state.osd_chan12_raw;
  channel_values[13] = osdproc_osd_state.osd_chan13_raw;
  channel_values[14] = osdproc_osd_state.osd_chan14_raw;
  channel_values[15] = osdproc_osd_state.osd_chan15_raw;
  channel_values[16] = osdproc_osd_state.osd_chan16_raw;  
    
  // It is likely better to directly record the count of channels reported 
  // by a Mavlink message, but this will probably work as a starting point.
  int number_of_rc_channels = osdproc_osd_state.osd_chan_cnt_above_eight ? 16 : 8;
  
  int posX = eeprom_buffer.params.RC_Channels_posx;
  int posY = eeprom_buffer.params.RC_Channels_posy;
  
  // Could make this configurable I suppose?
  int bar_width = 40;
  
  int font_number_for_rc_channel_text = 0;
  struct FontEntry font_info;
  fetch_font_info(0, font_number_for_rc_channel_text, &font_info, NULL);
  struct FontDimensions text_dim;
  
  // Go through each of the RC Channels. Again, note that channel index starts at 1, not 0.
  for (uint32_t channel_index = 1; channel_index <= number_of_rc_channels; channel_index++) { 
    uint32_t current_channel_value = channel_values[channel_index];
    
    // Write as text value    
    // --------------------
    sprintf(tmp_str, "CH %d %4d", (int)channel_index, current_channel_value);
    calc_text_dimensions(tmp_str, font_info, 1, 0, &text_dim);               
    int line_y_offset = (channel_index-1) * text_dim.height;
    int line_y_pos = posY + line_y_offset;
    write_string(tmp_str, posX, line_y_pos, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
    
    // Also draw as a bar graphic   
    // --------------------------
    int bar_pos_x = posX + text_dim.width;
    int bar_pos_y = line_y_pos;
    
    int bar_height = text_dim.height - (int)(text_dim.height * 0.20);
    
    // Draw rectangle for bar
     write_filled_rectangle_lm(bar_pos_x, bar_pos_y, bar_width, bar_height, 1, 0);
     write_rectangle_outlined(bar_pos_x, bar_pos_y, bar_width, bar_height, 0, 1);
     // Here we write an extra line at each end; this keeps the 3-pixel wide stripe
     // from writing outside the boundaries of the box below.
     write_rectangle_outlined(bar_pos_x-1, bar_pos_y, bar_width+1, bar_height, 0, 1);

    // Normalize 1000-2000 PPM value to 0-1000
    int normalized_channel_value = current_channel_value - 1000;
    if (normalized_channel_value < 0) {
        normalized_channel_value = 0;
    } else if (normalized_channel_value > 1000) {
        normalized_channel_value = 1000;
    }

    // Draw vertical stripe marking channel value on bar rectangle
    
    // X offset position for the stripe relative to the bar
    int stripe_offset_x = (normalized_channel_value * bar_width) / 1000;    

    // Stripe is 3 pixels wide
    write_vline_lm(bar_pos_x + stripe_offset_x - 1, bar_pos_y, bar_pos_y + bar_height, 1, 1);    
    write_vline_lm(bar_pos_x + stripe_offset_x, bar_pos_y, bar_pos_y + bar_height, 1, 1);    
    write_vline_lm(bar_pos_x + stripe_offset_x + 1, bar_pos_y, bar_pos_y + bar_height, 1, 1);          
  }  
}  

void draw_wind(void) {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Wind_en,
                              eeprom_buffer.params.Wind_panel)) {
    return;
  }

  char tmp_str[10] = { 0 };
  uint16_t posX = eeprom_buffer.params.Wind_posX;
  uint16_t posY = eeprom_buffer.params.Wind_posY;

  //write_string("wind:", posX, posY, 0, 0, TEXT_VA_MIDDLE, eeprom_buffer.params.Wind_align, 0, SIZE_TO_FONT[eeprom_buffer.params.Wind_fontsize]);

  //draw direction
  POLYGON2D obj2D;
  obj2D.state       = 1;
  obj2D.num_verts   = 5;
  obj2D.x0          = posX;
  obj2D.y0          = posY;
  VECTOR2D_INITXYZ(&(obj2D.vlist_local[0]), -3, -2);
  VECTOR2D_INITXYZ(&(obj2D.vlist_local[1]), 0, -8);
  VECTOR2D_INITXYZ(&(obj2D.vlist_local[2]), 3, -2);
  VECTOR2D_INITXYZ(&(obj2D.vlist_local[3]), 0, 8);
  VECTOR2D_INITXYZ(&(obj2D.vlist_local[4]), 0, -2);
  Reset_Polygon2D(&obj2D);
  Rotate_Polygon2D(&obj2D, osdproc_osd_state.osd_windDir);
  write_triangle_wire(obj2D.vlist_trans[0].x + obj2D.x0, obj2D.vlist_trans[0].y + obj2D.y0,
                      obj2D.vlist_trans[1].x + obj2D.x0, obj2D.vlist_trans[1].y + obj2D.y0,
                      obj2D.vlist_trans[2].x + obj2D.x0, obj2D.vlist_trans[2].y + obj2D.y0);
  write_line_outlined(obj2D.vlist_trans[3].x + obj2D.x0, obj2D.vlist_trans[3].y + obj2D.y0,
                      obj2D.vlist_trans[4].x + obj2D.x0, obj2D.vlist_trans[4].y + obj2D.y0, 2, 2, 0, 1);

  //draw wind speed
  float tmp = osdproc_osd_state.osd_windSpeed * convert_speed;
  sprintf(tmp_str, "%.2f%s", tmp, spd_unit);
  write_string(tmp_str, posX + 15, posY, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
}

/*
int debug_warnings_x = 30;
int debug_warnings_y = 30;

void debug_warnings_one() {    
    sprintf(tmp_str, "GetSystimeMS(): %d", (int)GetSystimeMS());
    write_string(tmp_str, debug_warnings_x, debug_warnings_y, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
    
    sprintf(tmp_str, "last_warn_time: %d", (int)last_warn_time);
    write_string(tmp_str, debug_warnings_x, debug_warnings_y + 15, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
}

void debug_warnings_two() {    
    sprintf(tmp_str, "osd_fix_type: %d", osd_fix_type);
    write_string(tmp_str, debug_warnings_x, debug_warnings_y + 30, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);    
}

void debug_warnings_three(uint8_t warning[], const int warn_cnt) {    
    sprintf(tmp_str, "last_warn_type: %d", last_warn_type);
    write_string(tmp_str, debug_warnings_x, debug_warnings_y + 45, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);        
    
    for (int i = 0; i < warn_cnt; i++){
        sprintf(tmp_str, "w[0]: %d", warning[i]);
        write_string(tmp_str, debug_warnings_x, debug_warnings_y + 60 + (i * 15), 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);       
    }
}
*/

void draw_warning(void) {
  write_string(warn_str, eeprom_buffer.params.Alarm_posX, eeprom_buffer.params.Alarm_posY, 0, 0, TEXT_VA_TOP, eeprom_buffer.params.Alarm_align, 0, SIZE_TO_FONT[eeprom_buffer.params.Alarm_fontsize]);

  // Show each warning a given number of milliseconds
  if ((GetSystimeMS() - last_warn_time) < eeprom_buffer.params.error_alert_milliseconds_to_show) {
    //debug_warnings_one();
    return;
  }

  bool haswarn = false;
  const static int warn_cnt = 6;
  uint8_t warning[] = { 0, 0, 0, 0, 0, 0, 0 };

  //no GPS fix!
  if (eeprom_buffer.params.Alarm_GPS_status_en == 1 && (osdproc_osd_state.osd_fix_type < GPS_OK_FIX_3D)) {
    //debug_warnings_two();
    haswarn = true;
    warning[0] = 1;
  }

  //low batt
  if (eeprom_buffer.params.Alarm_low_batt_en == 1 && (osdproc_osd_state.osd_battery_remaining_A < eeprom_buffer.params.Alarm_low_batt)) {
    haswarn = true;
    warning[1] = 1;
  }

  float spd_comparison = osdproc_osd_state.osd_groundspeed;
  if (eeprom_buffer.params.Spd_Scale_type == 1) {
    spd_comparison = osdproc_osd_state.osd_airspeed;
  }
  spd_comparison *= convert_speed;
  //under speed
  if (eeprom_buffer.params.Alarm_low_speed_en == 1 && (spd_comparison < eeprom_buffer.params.Alarm_low_speed)) {
    haswarn = true;
    warning[2] = 1;
  }

  //over speed
  if (eeprom_buffer.params.Alarm_over_speed_en == 1 && (spd_comparison > eeprom_buffer.params.Alarm_over_speed)) {
    haswarn = true;
    warning[3] = 1;
  }

  // float osd_alt = 0.0f;
  // get_osd_alt(&osd_alt);
  float osd_alt = osdproc_osd_state.osd_alt;  
    
  float alt_comparison = osdproc_osd_state.osd_rel_alt;
  if (eeprom_buffer.params.Alt_Scale_type == 0) {
    alt_comparison = osd_alt;
  }
  //under altitude
  if (eeprom_buffer.params.Alarm_low_alt_en == 1 && (alt_comparison < eeprom_buffer.params.Alarm_low_alt)) {
    haswarn = true;
    warning[4] = 1;
  }

  //over altitude
  if (eeprom_buffer.params.Alarm_over_alt_en == 1 && (alt_comparison > eeprom_buffer.params.Alarm_over_alt)) {
    haswarn = true;
    warning[5] = 1;
  }

  // no home yet
  if (get_osd_got_home() == 0) {
    haswarn = true;
    warning[6] = 1;
  }

  if (haswarn) {
    last_warn_time = GetSystimeMS();
    // Wrap back to the start of the error type array
    if (last_warn_type > warn_cnt) {
        last_warn_type = 0;
    }
    
    //debug_warnings_three(warning, warn_cnt);

    if (last_warn_type == 0) {
      last_warn_type++;
      if (warning[0] == 1) {
          warn_str = "NO GPS FIX";
          return;
      }
    }
    
    if (last_warn_type == 1) {
      last_warn_type++;
      if (warning[1] == 1) {
          warn_str = "LOW BATTERY";
          return;
      }
    }
    
    if (last_warn_type == 2) {
      last_warn_type++;
      if (warning[2] == 1) {
          warn_str = "SPEED LOW";
          return;
      }
    }
    
    if (last_warn_type == 3) {
      last_warn_type++;
      if (warning[3] == 1) {
          warn_str = "OVER SPEED";
          return;
      }
    }
    
    if (last_warn_type == 4) {
      last_warn_type++;
      if (warning[4] == 1) {
          warn_str = "LOW ALT";
          return;
      }
    }
    
    if (last_warn_type == 5) {
      last_warn_type++;
      if (warning[5] == 1) {
          warn_str = "HIGH ALT";
          return;
      }
    }
    
    if (last_warn_type == 6) {
      last_warn_type++;
      if (warning[6] == 1) {
          warn_str = "NO HOME POSITION SET";
          return;
      }
    }

    // If we didn't match, there's a coding
    // error of some kind. Warn the user/developer.
    warn_str = "ERROR IN WARNING ROUTINE";
    return;
  }

  // If you haven't found an error, this should ALWAYS get cleared
  warn_str = "";
}

void debug_wps(void) {
  char tmp_str[50] = { 0 };

//    //debug
//    uint16_t a = 20;
//    for(int i=1; i<wp_counts; i++)
//    {
//        sprintf(tmp_str, "WP%d X:%0.12f Y:%0.12f",
//                         (int)wp_list[i].seq, (double)wp_list[i].x,
//                          (double)wp_list[i].y);
//        write_string(tmp_str, 10, a, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
//        a += 15;
//    }

//    float uav_lat = osd_lat / DEGREE_MULTIPLIER;
//    float uav_lon = osd_lon / DEGREE_MULTIPLIER;
  float home_lat = get_osd_home_lat() / DEGREE_MULTIPLIER;
  float home_lon = get_osd_home_lon() / DEGREE_MULTIPLIER;

//    if(osd_fix_type > 1){
//        sprintf(tmp_str, "UAV X:%0.12f Y:%0.12f",(double)uav_lat,(double)uav_lon);
//        write_string(tmp_str, 10, a, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
//        a += 15;
//    }
  
  if (get_osd_got_home() == 1) {
//        sprintf(tmp_str, "home X:%0.12f Y:%0.12f",(double)home_lat,(double)home_lon);
//        write_string(tmp_str, 10, a, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
//        a += 15;
    sprintf(tmp_str, "%0.5f %0.5f", (double)home_lat, (double)home_lon);
    write_string(tmp_str, 70, 210, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);
  }

  return;
}

void gen_overlay_rect(float lat, float lon, VECTOR4D_PTR vec) {
  //left:vec.x  top:vec.y  right:vec.z  bottom:vec.w
  if (lon < vec->x) vec->x = lon;
  if (lat > vec->y) vec->y = lat;
  if (lon > vec->z) vec->z = lon;
  if (lat < vec->w) vec->w = lat;
}

VERTEX2DF gps_to_screen_pixel(float lat, float lon, float cent_lat, float cent_lon,
                              float rect_diagonal_half, float cent_x, float cent_y, float radius) {
  float dstlon, dstlat, distance;
  float scaleLongUp, scaleLongDown;
  int dir = 0;

  VERTEX2DF point_ret;

  scaleLongUp   = 1.0f / Fast_Cos(fabs(lat));
  scaleLongDown = Fast_Cos(fabs(lat));

  dstlon = fabs(lon - cent_lon) * 111319.5f * scaleLongDown;
  dstlat = fabs(lat - cent_lat) * 111319.5f;
  distance = sqrt(dstlat * dstlat + dstlon * dstlon);

  dstlon = (lon - cent_lon);
  dstlat = (lat - cent_lat) * scaleLongUp;
  dir = 270 + (atan2(dstlat, -dstlon) * R2D);
  dir = (dir + 360) % 360;
  point_ret.x = cent_x + radius * Fast_Sin(dir) * distance / rect_diagonal_half;
  point_ret.y = cent_y - radius * Fast_Cos(dir) * distance / rect_diagonal_half;

  return point_ret;
}

void draw_flight_mode() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.FlightMode_en,
                              eeprom_buffer.params.FlightMode_panel)) {
    return;
  }

  char* mode_str = "UNKNOWN";
  switch (osdproc_osd_state.autopilot) {
  case 3:       //ardupilotmega
  {
    uint32_t custom_mode = osdproc_osd_state.custom_mode;
    uint8_t mav_type = get_mav_type();
    
    if (mav_type != 1) {
      if (custom_mode == 0)       mode_str = "STAB";              //manual airframe angle with manual throttle
      else if (custom_mode == 1)  mode_str = "ACRO";              //manual body-frame angular rate with manual throttle
      else if (custom_mode == 2)  mode_str = "ALTH";              //manual airframe angle with automatic throttle
      else if (custom_mode == 3)  mode_str = "AUTO";              //fully automatic waypoint control using mission commands
      else if (custom_mode == 4)  mode_str = "GUID";              //fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
      else if (custom_mode == 5)  mode_str = "LOIT";              //automatic horizontal acceleration with automatic throttle
      else if (custom_mode == 6)  mode_str = "RETL";              //automatic return to launching point
      else if (custom_mode == 7)  mode_str = "CIRC";              //automatic circular flight with automatic throttle
      //else if (custom_mode == 8)  mode_str = "POSI"; //Position: auto control
      else if (custom_mode == 9)  mode_str = "LAND";              //automatic landing with horizontal position control
      else if (custom_mode == 10) mode_str = "OFLO";              //deprecated
      else if (custom_mode == 11) mode_str = "DRIF";              //semi-automous position, yaw and throttle control
      else if (custom_mode == 13) mode_str = "SPRT";              //manual earth-frame angular rate control with manual throttle
      else if (custom_mode == 14) mode_str = "FLIP";              //automatically flip the vehicle on the roll axis
      else if (custom_mode == 15) mode_str = "ATUN";              //automatically tune the vehicle's roll and pitch gains
      else if (custom_mode == 16) mode_str = "POSH";              //automatic position hold with manual override, with automatic throttle
      else if (custom_mode == 17) mode_str = "BRAK";              //full-brake using inertial/GPS system, no pilot input
    } else if (mav_type == 1) {          //ArduPlane
      if (custom_mode == 0)       mode_str = "MANU";              //Manual
      else if (custom_mode == 1)  mode_str = "CIRC";              //Circle
      else if (custom_mode == 2)  mode_str = "STAB";              //Stabilize
      else if (custom_mode == 3)  mode_str = "TRNG";              //Training
      else if (custom_mode == 4)  mode_str = "ACRO";              //Acro
      else if (custom_mode == 5)  mode_str = "FBWA";              //Fly_By_Wire_A
      else if (custom_mode == 6)  mode_str = "FBWB";              //Fly_By_Wire_B
      else if (custom_mode == 7)  mode_str = "CRUI";              //Cruise
      else if (custom_mode == 8)  mode_str = "ATUN";              //Auto Tune
      else if (custom_mode == 10) mode_str = "AUTO";              //Auto
      else if (custom_mode == 11) mode_str = "RETL";              //Return to Launch
      else if (custom_mode == 12) mode_str = "LOIT";              //Loiter
      else if (custom_mode == 15) mode_str = "GUID";              //Guided
      else if (custom_mode == 16) mode_str = "INIT";              //Initializing
    }

    break;
  }
  case 12:       //PX4
  {
    union px4_custom_mode custom_mode_px4;
    custom_mode_px4.data = osdproc_osd_state.custom_mode;

    if (custom_mode_px4.main_mode == 1) mode_str = "MANUAL";
    else if (custom_mode_px4.main_mode == 2) mode_str = "ALTCTL";
    else if (custom_mode_px4.main_mode == 3) mode_str = "POSCTL";
    else if (custom_mode_px4.main_mode == 4)
    {
      if (custom_mode_px4.sub_mode == 1)  mode_str = "READY";
      else if (custom_mode_px4.sub_mode == 2)  mode_str = "TAKEOFF";
      else if (custom_mode_px4.sub_mode == 3)  mode_str = "LOITER";
      else if (custom_mode_px4.sub_mode == 4)  mode_str = "MISSION";
      else if (custom_mode_px4.sub_mode == 5)  mode_str = "RTL";
      else if (custom_mode_px4.sub_mode == 6)  mode_str = "LAND";
      else if (custom_mode_px4.sub_mode == 7)  mode_str = "RTGS";
      else if (custom_mode_px4.sub_mode == 8)  mode_str = "FOLLOW";
    }
    else if (custom_mode_px4.main_mode == 5) mode_str = "ACRO";
    else if (custom_mode_px4.main_mode == 6) mode_str = "OFFBOARD";
    else if (custom_mode_px4.main_mode == 7) mode_str = "STABILIZE";
    else if (custom_mode_px4.main_mode == 8) mode_str = "RATTITUDE";

    break;
  }

  default:
  {
    mode_str = "UNSUPPORTED";
  }
  }

  write_string(mode_str, eeprom_buffer.params.FlightMode_posX, eeprom_buffer.params.FlightMode_posY,
               0, 0, TEXT_VA_TOP, eeprom_buffer.params.FlightMode_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.FlightMode_fontsize]);
}

void draw_arm_state() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Arm_en,
                              eeprom_buffer.params.Arm_panel)) {
    return;
  }

  tmp_str1 = osdproc_osd_state.motor_armed ? "ARMED" : "DISARMED";
  write_string(tmp_str1, eeprom_buffer.params.Arm_posX,
               eeprom_buffer.params.Arm_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Arm_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Arm_fontsize]);
}

void draw_battery_voltage() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.BattVolt_en,
                              eeprom_buffer.params.BattVolt_panel)) {
    return;
  }

  sprintf(tmp_str, "%0.1fV", (double) osdproc_osd_state.osd_vbat_A);
  write_string(tmp_str, eeprom_buffer.params.BattVolt_posX,
               eeprom_buffer.params.BattVolt_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.BattVolt_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.BattVolt_fontsize]);
}

void draw_battery_current() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.BattCurrent_en,
                              eeprom_buffer.params.BattCurrent_panel)) {
    return;
  }

  sprintf(tmp_str, "%0.1fA", (double) (osdproc_osd_state.osd_curr_A * 0.01));
  write_string(tmp_str, eeprom_buffer.params.BattCurrent_posX,
               eeprom_buffer.params.BattCurrent_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.BattCurrent_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.BattCurrent_fontsize]);
}

void draw_battery_remaining() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.BattRemaining_en,
                              eeprom_buffer.params.BattRemaining_panel)) {
    return;
  }

  sprintf(tmp_str, "%d%%", osdproc_osd_state.osd_battery_remaining_A);
  write_string(tmp_str, eeprom_buffer.params.BattRemaining_posX,
               eeprom_buffer.params.BattRemaining_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.BattRemaining_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.BattRemaining_fontsize]);
}

void draw_battery_consumed() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.BattConsumed_en,
                              eeprom_buffer.params.BattConsumed_panel)) {
    return;
  }

  sprintf(tmp_str, "%dmah", (int)get_current_consumed_mah());
  write_string(tmp_str, eeprom_buffer.params.BattConsumed_posX,
               eeprom_buffer.params.BattConsumed_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.BattConsumed_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.BattConsumed_fontsize]);
}

void draw_altitude_scale() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Alt_Scale_en,
                              eeprom_buffer.params.Alt_Scale_panel)) {
    return;
  }

  float osd_alt = osdproc_osd_state.osd_alt;      
  float alt_shown = osdproc_osd_state.osd_rel_alt;
  
  uint16_t posX = eeprom_buffer.params.Alt_Scale_posX;
  sprintf(tmp_str, "Alt");
  if (eeprom_buffer.params.Alt_Scale_type == 0) {
    alt_shown = osd_alt;
    sprintf(tmp_str, "AAlt");
  }
  draw_vertical_scale(alt_shown * convert_distance, 60,
                      eeprom_buffer.params.Alt_Scale_align,
                      eeprom_buffer.params.Alt_Scale_posX,
                      eeprom_buffer.params.Alt_Scale_posY, 72, 10, 20, 5, 8, 11,
                      10000, 0);
  if ((eeprom_buffer.params.Alt_Scale_align == 1) && (posX > 15)) {
    posX -= 15;
  }
  write_string(tmp_str, posX,
               eeprom_buffer.params.Alt_Scale_posY - 50, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Alt_Scale_align, 0,
               SIZE_TO_FONT[0]);
  if ((eeprom_buffer.params.Alt_Scale_align == 1) && (posX > 15)) {
    posX += 10;
  }
  write_string("M", posX,
               eeprom_buffer.params.Alt_Scale_posY + 40, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Alt_Scale_align, 0,
               SIZE_TO_FONT[0]);
}

void draw_absolute_altitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.TALT_en,
                              eeprom_buffer.params.TALT_panel)) {
    return;
  }

  float osd_alt = osdproc_osd_state.osd_alt;     
  float tmp = osd_alt * convert_distance;

  if (tmp < convert_distance_divider) {
    sprintf(tmp_str, "AA %d%s", (int) tmp, dist_unit_short);
  }
  else{
    sprintf(tmp_str, "AA %0.2f%s", (double) (tmp / convert_distance_divider), dist_unit_long);
  }

  write_string(tmp_str, eeprom_buffer.params.TALT_posX,
               eeprom_buffer.params.TALT_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.TALT_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.TALT_fontsize]);
}

void draw_relative_altitude() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Relative_ALT_en,
                              eeprom_buffer.params.Relative_ALT_panel)) {
    return;
  }

  float tmp = osdproc_osd_state.osd_rel_alt * convert_distance;
  if (tmp < convert_distance_divider) {
    sprintf(tmp_str, "A %d%s", (int) tmp, dist_unit_short);
  }
  else{
    sprintf(tmp_str, "A %0.2f%s", (double) (tmp / convert_distance_divider), dist_unit_long);
  }

  write_string(tmp_str, eeprom_buffer.params.Relative_ALT_posX,
               eeprom_buffer.params.Relative_ALT_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Relative_ALT_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Relative_ALT_fontsize]);
}

void draw_speed_scale() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Speed_scale_en,
                              eeprom_buffer.params.Speed_scale_panel)) {
    return;
  }

  float spd_shown = osdproc_osd_state.osd_groundspeed;
  sprintf(tmp_str, "GS");
  if (eeprom_buffer.params.Spd_Scale_type == 1) {
    spd_shown = osdproc_osd_state.osd_airspeed;
    sprintf(tmp_str, "AS");
  }
  draw_vertical_scale(spd_shown * convert_speed, 60,
                      eeprom_buffer.params.Speed_scale_align,
                      eeprom_buffer.params.Speed_scale_posX,
                      eeprom_buffer.params.Speed_scale_posY, 72, 10, 20, 5, 8, 11,
                      100, 0);
  write_string(tmp_str, eeprom_buffer.params.Speed_scale_posX,
               eeprom_buffer.params.Speed_scale_posY - 50, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Speed_scale_align, 0,
               SIZE_TO_FONT[0]);

  sprintf(tmp_str, "%s", spd_unit);
  write_string(tmp_str, eeprom_buffer.params.Speed_scale_posX,
               eeprom_buffer.params.Speed_scale_posY + 40, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Speed_scale_align, 0,
               SIZE_TO_FONT[0]);
}

void draw_ground_speed() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.TSPD_en,
                              eeprom_buffer.params.TSPD_panel)) {
    return;
  }

  float tmp = osdproc_osd_state.osd_groundspeed * convert_speed;
  sprintf(tmp_str, "GS %d%s", (int) tmp, spd_unit);
  write_string(tmp_str, eeprom_buffer.params.TSPD_posX,
               eeprom_buffer.params.TSPD_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.TSPD_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.TSPD_fontsize]);
}

void draw_air_speed() {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Air_Speed_en,
                              eeprom_buffer.params.Air_Speed_panel)) {
    return;
  }

  float tmp = osdproc_osd_state.osd_airspeed * convert_speed;
  sprintf(tmp_str, "AS %d%s", (int) tmp, spd_unit);
  write_string(tmp_str, eeprom_buffer.params.Air_Speed_posX,
               eeprom_buffer.params.Air_Speed_posY, 0, 0, TEXT_VA_TOP,
               eeprom_buffer.params.Air_Speed_align, 0,
               SIZE_TO_FONT[eeprom_buffer.params.Air_Speed_fontsize]);
}

// Port to mutex-land of original draw_map code
void draw_map(void) {
  if (!enabledAndShownOnPanel(eeprom_buffer.params.Map_en,
                              eeprom_buffer.params.Map_panel)) {
    return;
  }

  if (osdproc_osd_state.got_all_wps == 0) {
    return;  
  }    

  uint8_t osd_got_home = get_osd_got_home();  
  
  char tmp_str[50] = { 0 };

  float osd_lat_current = osdproc_osd_state.osd_lat;
  float osd_lon_current = osdproc_osd_state.osd_lon; 
  
  float uav_lat = osd_lat_current / DEGREE_MULTIPLIER;
  float uav_lon = osd_lon_current / DEGREE_MULTIPLIER;
  float home_lat = get_osd_home_lat() / DEGREE_MULTIPLIER;
  float home_lon = get_osd_home_lon() / DEGREE_MULTIPLIER;

  float uav_x = 0.0f;
  float uav_y = 0.0f;

  VECTOR4D rect;
  rect.x = 999.0f;
  rect.y = -999.0f;
  rect.z = -999.0f;
  rect.w = 999.0f;

  for (int i = 1; i < osdproc_osd_state.wp_counts; i++) {
    gen_overlay_rect(osdproc_osd_state.wp_list[i].x, osdproc_osd_state.wp_list[i].y, &rect);
  }

  if (osdproc_osd_state.osd_fix_type > 1) {
    gen_overlay_rect(uav_lat, uav_lon, &rect);
  }

  if (osd_got_home == 1) {
    gen_overlay_rect(home_lat, home_lon, &rect);
  }

  float rect_half_height = fabs(rect.y - rect.w) / 2;
  float rect_half_width = fabs(rect.x - rect.z) / 2;
  float cent_lat = rect.w + rect_half_height;
  float cent_lon = rect.x + rect_half_width;
  uint32_t cent_x = GRAPHICS_X_MIDDLE;
  uint32_t cent_y = GRAPHICS_Y_MIDDLE;

  uint32_t radius = eeprom_buffer.params.Map_radius;
  if (radius < 1) radius = 1;
  if (radius > 120) radius = 120;

  float dstlon, dstlat, scaleLongDown;

  VERTEX2DF wps_screen_point[osdproc_osd_state.wp_counts];
  scaleLongDown = Fast_Cos(fabs(rect.y));
  dstlon = fabs(rect.x - rect.z) * 111319.5f * scaleLongDown;
  dstlat = fabs(rect.y - rect.w) * 111319.5f;
  float rect_diagonal_half = sqrt(dstlat * dstlat + dstlon * dstlon) / 2;

  VERTEX2DF tmp_point;
  
  // Translate to screen coordinates for waypoints (if any)
  for (int i = 1; i < osdproc_osd_state.wp_counts; i++) {
    wps_screen_point[i] = gps_to_screen_pixel(osdproc_osd_state.wp_list[i].x, osdproc_osd_state.wp_list[i].y, cent_lat, cent_lon,
                                              rect_diagonal_half, cent_x, cent_y, radius);
  }

  // Translate to screen coordinates for UAV (if GPS location known)
  if (osdproc_osd_state.osd_fix_type > 1) {
    tmp_point = gps_to_screen_pixel(uav_lat, uav_lon, cent_lat, cent_lon,
                                    rect_diagonal_half, cent_x, cent_y, radius);
    uav_x = tmp_point.x;
    uav_y = tmp_point.y;
  }
  
  // Translate to screen coordinates for home (if home is set)
  // Note that home is [0] in the wps_screen_point array, and waypoints are [1] - [X].
  if (osd_got_home == 1) {
    wps_screen_point[0] = gps_to_screen_pixel(home_lat, home_lon, cent_lat, cent_lon,
                                              rect_diagonal_half, cent_x, cent_y, radius);
  }

  // Draw lines between all waypoints (if any)
  for (int i = 1; i < osdproc_osd_state.wp_counts - 1; i++) {
    write_line_outlined(wps_screen_point[i].x, wps_screen_point[i].y, wps_screen_point[i + 1].x, wps_screen_point[i + 1].y, 2, 2, 0, 1);
  }

  // If home is set, and we have at least one waypoint, draw line between home and first waypoint
  if (osd_got_home == 1 && osdproc_osd_state.wp_counts > 1) {
    write_line_outlined(wps_screen_point[0].x, wps_screen_point[0].y, wps_screen_point[1].x, wps_screen_point[1].y, 2, 2, 0, 1);
  }

  // Draw waypoint numbers (if any)
  for (int i = 1; i < osdproc_osd_state.wp_counts; i++) {
    sprintf(tmp_str, "%d", osdproc_osd_state.wp_list[i].seq);
    write_string(tmp_str, wps_screen_point[i].x, wps_screen_point[i].y, 0, 0, eeprom_buffer.params.Map_V_align, eeprom_buffer.params.Map_H_align, 0, SIZE_TO_FONT[eeprom_buffer.params.Map_fontsize]);
  }

  // Draw home (if home is set)
  if (osd_got_home == 1) {
    write_string("H", wps_screen_point[0].x, wps_screen_point[0].y, 0, 0, eeprom_buffer.params.Map_V_align, eeprom_buffer.params.Map_H_align, 0, SIZE_TO_FONT[eeprom_buffer.params.Map_fontsize]);
  }
  
  // Draw UAV (if GPS location known)
  if (osdproc_osd_state.osd_fix_type > 1) {
    //draw heading
    POLYGON2D suav;
    suav.state       = 1;
    suav.num_verts   = 3;
    suav.x0          = uav_x;
    suav.y0          = uav_y;
    VECTOR2D_INITXYZ(&(suav.vlist_local[0]), 0, -8);
    VECTOR2D_INITXYZ(&(suav.vlist_local[1]), -5, 8);
    VECTOR2D_INITXYZ(&(suav.vlist_local[2]), 5, 8);
    Reset_Polygon2D(&suav);
    Rotate_Polygon2D(&suav, osdproc_osd_state.osd_heading);
    write_line_outlined(suav.vlist_trans[0].x + suav.x0, suav.vlist_trans[0].y + suav.y0,
                        suav.vlist_trans[1].x + suav.x0, suav.vlist_trans[1].y + suav.y0, 2, 2, 0, 1);
    write_line_outlined(suav.vlist_trans[0].x + suav.x0, suav.vlist_trans[0].y + suav.y0,
                        suav.vlist_trans[2].x + suav.x0, suav.vlist_trans[2].y + suav.y0, 2, 2, 0, 1);
  }
}

/*
void DJI_test(void) {
  char tmp_str[100] = { 0 };

  int16_t tmp_int16;
  int tmp_int1, tmp_int2;

  sprintf(tmp_str, "%0.1fV", (float)osd_vbat_A);
  write_string(tmp_str, 20, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "%i", osd_battery_remaining_A);
  write_string(tmp_str, 90, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "P %d", (int32_t)osd_pitch);
  write_string(tmp_str, GRAPHICS_X_MIDDLE - 50, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "R %d", (int32_t)osd_roll);
  write_string(tmp_str, GRAPHICS_X_MIDDLE - 50, 65, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "hV %0.1f", (double)osd_groundspeed);
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 10, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "GPS %d", osd_satellites_visible);
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 150, 50, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "%0.5f lat", (double)(osd_lat / 10000000.0));
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 170, 65, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "%0.5f lon", (double)(osd_lon / 10000000.0));
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 170, 80, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "%0.1f hdop", (double)(osd_hdop));
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 170, 95, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "D %d", (int32_t)osd_home_distance);
  write_string(tmp_str, 20, GRAPHICS_Y_MIDDLE - 30, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "H %d", (int32_t)osd_alt);
  write_string(tmp_str, 20, GRAPHICS_Y_MIDDLE - 15, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[1]);

  char* mode_str = "unknown";
  if (custom_mode == 0) mode_str = "MANUAL";
  else if (custom_mode == 1) mode_str = "GPS";
  else if (custom_mode == 2) mode_str = "FAILSAFE";
  else if (custom_mode == 3) mode_str = "ATTI";

  write_string(mode_str, 20, GRAPHICS_Y_MIDDLE + 15, 0, 0, TEXT_VA_TOP, TEXT_HA_LEFT, 0, SIZE_TO_FONT[0]);

  sprintf(tmp_str, "%0.5f Hlat", (double)(osd_home_lat / 10000000.0));
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 10, GRAPHICS_Y_MIDDLE + 15, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);
  sprintf(tmp_str, "%0.5f Hlon", (double)(osd_home_lon / 10000000.0));
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 170, GRAPHICS_Y_MIDDLE + 15, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);

  sprintf(tmp_str, "vV %0.1f", (double)osd_downVelocity);
  write_string(tmp_str, GRAPHICS_X_MIDDLE + 150, GRAPHICS_Y_MIDDLE - 15, 0, 0, TEXT_VA_TOP, TEXT_HA_RIGHT, 0, SIZE_TO_FONT[1]);

  draw_linear_compass(osd_heading, osd_home_bearing, 120, 180, GRAPHICS_X_MIDDLE, GRAPHICS_Y_MIDDLE + 80, 15, 30, 5, 8, 0);
}
*/