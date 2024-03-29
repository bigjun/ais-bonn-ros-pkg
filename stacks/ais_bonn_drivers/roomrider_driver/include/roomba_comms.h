/*
 * roomba_comms.h
 *
 *
 * Copyright (C) 2010
 * Autonomous Intelligent Systems Group
 * University of Bonn, Germany
 *
 *
 * Authors: Andreas Hochrath, Torsten Fiolka
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * Origin:
 *
 *  iRobot Create serial communications C++ interface
 *  taken from Player source, modified by Chad Jenkins (2009)
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2006 -
 *     Brian Gerkey
 *
 */

#ifndef roomba_comms
#define roomba_comms

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <limits.h>
#include <stdlib.h> // cjenkins
#include <stdint.h> // cjenkins
#include <sys/poll.h>

/*! command opcodes from the specification */
#define ROOMBA_OPCODE_START            128
#define ROOMBA_OPCODE_BAUD             129
#define ROOMBA_OPCODE_SAFE             131
#define ROOMBA_OPCODE_FULL             132
#define ROOMBA_OPCODE_SPOT             134
#define ROOMBA_OPCODE_COVER            135
#define ROOMBA_OPCODE_DEMO             136
#define ROOMBA_OPCODE_DRIVE            137
#define ROOMBA_OPCODE_MOTORS           138
#define ROOMBA_OPCODE_LEDS             139
#define ROOMBA_OPCODE_SONG             140
#define ROOMBA_OPCODE_PLAY             141
#define ROOMBA_OPCODE_SENSORS          142
#define ROOMBA_OPCODE_COVERDOCK        143
#define ROOMBA_OPCODE_PWM_MOTORS       144
#define ROOMBA_OPCODE_DRIVE_WHEELS     145
#define ROOMBA_OPCODE_DIGITAL_OUTPUTS  147
#define ROOMBA_OPCODE_STREAM           148
#define ROOMBA_OPCODE_QUERY_LIST       149
#define ROOMBA_OPCODE_DO_STREAM        150
#define ROOMBA_OPCODE_SEND_IR_CHAR     151
#define ROOMBA_OPCODE_SCRIPT           152
#define ROOMBA_OPCODE_PLAY_SCRIPT      153
#define ROOMBA_OPCODE_SHOW_SCRIPT      154
#define ROOMBA_OPCODE_WAIT_TIME        155
#define ROOMBA_OPCODE_WAIT_DISTANCE    156
#define ROOMBA_OPCODE_WAIT_ANGLE       157
#define ROOMBA_OPCODE_WAIT_EVENT       158

#define ROOMBA_DELAY_MODECHANGE_MS      20

#define ROOMBA_MODE_OFF                  0
#define ROOMBA_MODE_PASSIVE              1
#define ROOMBA_MODE_SAFE                 2
#define ROOMBA_MODE_FULL                 3

#define ROOMBA_TVEL_MAX_MM_S           500
#define ROOMBA_RADIUS_MAX_MM          2000

#define ROOMBA_SENSOR_PACKET_SIZE       80
#define ROOMBA_CHARGING_NOT              0
#define ROOMBA_CHARGING_RECOVERY         1
#define ROOMBA_CHARGING_CHARGING         2
#define ROOMBA_CHARGING_TRICKLE          3
#define ROOMBA_CHARGING_WAITING          4
#define ROOMBA_CHARGING_ERROR            5

#define ROOMBA_AXLE_LENGTH            0.258

#define ROOMBA_DIAMETER               0.33

#define TICKS_PER_MM		      2.278199548
#define ROOMBA_WHEEL_DISTANCE	      0.23456

#define ROOMBA_BUMPER_XOFFSET         0.05

#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif
#define DTOR(d) ((d) * M_PI / 180)

/**
 * \struct roomba500_comm_t
 * @brief This structure stores all data received from the roomba
 */
typedef struct
{
  /* Serial port to which the robot is connected */
  char serial_port[PATH_MAX];
  /* File descriptor associated with serial connection (-1 if no valid
   * connection) */
  int fd;
  /* Current operation mode - one of ROOMBA_MODE_* */
  unsigned char mode;
  /* Integrated odometric position [m m rad], currently unused */
  double ox, oy, oa;
  /* odometry state calculated from encoder-counts */
  double poseX, poseY, poseA, distance, poseVX, poseVY, pose_deltaA;

  /* Various Boolean flags */
  int bumper_left, bumper_right;
  unsigned char wheeldrop_left, wheeldrop_right;
  unsigned char wall;
  unsigned char cliff_left, cliff_frontleft, cliff_frontright, cliff_right;
  unsigned char virtual_wall;
  unsigned char button_dock, button_spot, button_clean;
  // the following are not tested
  unsigned char overcurrent_driveleft, overcurrent_driveright;
  unsigned char overcurrent_mainbrush, overcurrent_sidebrush;
  unsigned char overcurrent_vacuum;
  unsigned char remote_opcode;

  /* One of ROOMBA_CHARGING_* */
  unsigned char charging_state;

  // Battery sensors
  double voltage;
  double current;
  double temperature;
  double charge;
  double capacity;

  /* Raw values from the cliff sensors */
  unsigned short wall_data, cliff_left_data, cliff_frontleft_data, cliff_frontright_data, cliff_right_data;
  /* Connected to home base or internal charger */
  unsigned char home_base, internal_charger;
  /* current robot mode */
  unsigned char open_interface_mode;
  /* current song, currently not used */
  unsigned char song_number, song_playing;
  unsigned char oi_stream_num_packets;
  /* Raw odometry values, currently unused */
  short velocity, radius, velocity_right, velocity_left;

  /* Encoder counts from the wheels */
  unsigned short encoder_counts_left, encoder_counts_right;
  /* Encoder count offsets; needed for correct encoder_counts after the driver is
   * restart without the roomba being reset
   */
  unsigned short encoder_counts_left_offset, encoder_counts_right_offset;

  /* Light Bumper boolean Flags */
  unsigned char light_bump_right, light_bump_frontright, light_bump_centerright;
  unsigned char light_bump_centerleft, light_bump_frontleft, light_bump_left;

  /* Light Bumper raw data */
  unsigned short light_bump_right_data, light_bump_frontright_data, light_bump_centerright_data;
  unsigned short light_bump_centerleft_data, light_bump_frontleft_data, light_bump_left_data;

  /* Received infrared characters */
  unsigned char ir_character_left, ir_character_right;
  /* Motor currents, currently not used */
  short left_motor_current, right_motor_current, main_brush_current, side_brush_current;
  /* Stasis sensor, currently not used */
  unsigned char stasis;

} roomba500_comm_t;

//! Initializes the data structure
roomba500_comm_t* roomba500_create(const char* serial_port);

void roomba500_destroy(roomba500_comm_t* r);

//! Connects to the roomba and tests the connection
int roomba500_open(roomba500_comm_t* r, unsigned char fullcontrol);

//! Initializes the roomba in safe or full mode
int roomba500_init(roomba500_comm_t* r, unsigned char fullcontrol);

//! Stops the roomba and closes the connection
int roomba500_close(roomba500_comm_t* r);

int roomba500_set_speeds(roomba500_comm_t* r, double tv, double rv);

//! Reads raw sensor data from the roomba
int roomba500_get_sensors(roomba500_comm_t* r, int timeout);

//! Formats the raw sensor data an writes it into the roomba500_comm_t structure
int roomba500_parse_sensor_packet(roomba500_comm_t* r, unsigned char* buf, size_t buflen);

//! Verbose mode for Debugging
void roomba500_print(roomba500_comm_t* r);

//! Currently not used by the driver
int roomba500_set_song(roomba500_comm_t* r, unsigned char songNumber, unsigned char songLength, unsigned char *notes,
                       unsigned char *noteLengths);
//! Currently not used by the driver
int roomba500_play_song(roomba500_comm_t *r, unsigned char songNumber);

//! Currently not used by the driver
int roomba500_vacuum(roomba500_comm_t *r, int state);

int roomba500_set_leds(roomba500_comm_t *r, uint8_t dirt_detect, uint8_t dock, uint8_t power_color,
                       uint8_t power_intensity, uint8_t spot, uint8_t status);

//! Runs the internal roomba demo
int roomba500_run_demo(roomba500_comm_t *r, uint8_t num);

#ifdef __cplusplus
}
#endif

#endif /* roomba_comms */

