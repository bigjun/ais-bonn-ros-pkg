/*
 * roomba_comms.c
 *
 * modified by Andreas Hochrath & Torsten Fiolka in August 2010
 *
 * at	        University Bonn,
 * 		Institute for Computer Science VI,
 * 		Autonomous Intelligent Systems
 *
 * contact: fiolka@cs.uni-bonn.de, hochrath@cs.uni-bonn.de
 *
 * based on the roomba_comms.c from the Player/Stage project
 *
 * original header follows:
 */

/*
 *  iRobot Create serial communications C++ interface
 *  taken from Player source, modified by Chad Jenkins (2009)
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2006 -
 *     Brian Gerkey
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "roomba_comms.h"

roomba500_comm_t*
roomba500_create(const char* serial_port)
{
  roomba500_comm_t* r;

  r = (roomba500_comm_t*)calloc(1, sizeof(roomba500_comm_t));
  assert(r);
  r->fd = -1;
  r->mode = ROOMBA_MODE_OFF;

  strncpy(r->serial_port, serial_port, sizeof(r->serial_port) - 1);
  return (r);
}

void roomba500_destroy(roomba500_comm_t* r)
{
  free(r);
}

int roomba500_open(roomba500_comm_t* r, unsigned char fullcontrol)
{
  struct termios term;
  int flags;

  if (r->fd >= 0)
  {
    puts("create connection already open!");
    return (-1);
  }

  printf("Opening connection to Roomba on %s...", r->serial_port);
  fflush(stdout);

  // Open it.  non-blocking at first, in case there's no create
  if ((r->fd = open(r->serial_port, O_RDWR | O_NONBLOCK, S_IRUSR | S_IWUSR)) < 0)
  {
    fprintf(stdout, "opened port\n");
    perror("create_open():open():");
    return (-1);
  }

  if (tcflush(r->fd, TCIFLUSH) < 0)
  {
    fprintf(stdout, "tcflush\n");
    perror("create_open():tcflush():");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }
  if (tcgetattr(r->fd, &term) < 0)
  {
    perror("create_open():tcgetattr():");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }

  cfmakeraw(&term);
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(r->fd, TCSAFLUSH, &term) < 0)
  {
    perror("create_open():tcsetattr():");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }

  if (roomba500_init(r, fullcontrol) < 0)
  {
    puts("failed to initialize connection");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }

  if (roomba500_get_sensors(r, 100000) < 0)
  {
    puts("create_open():failed to get data");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }

  r->encoder_counts_left_offset = r->encoder_counts_left;
  r->encoder_counts_right_offset = r->encoder_counts_right;
  r->oa = r->ox = r->oy = 0;
  r->poseX = 0.0;
  r->poseY = 0.0;
  r->poseA = 0.0;
  r->poseVX = 0.0;
  r->poseVY = 0.0;

  // We know the robot is there; switch to blocking
  // ok, we got data, so now set NONBLOCK, and continue
  if ((flags = fcntl(r->fd, F_GETFL)) < 0)
  {
    perror("create_open():fcntl():");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }
  if (fcntl(r->fd, F_SETFL, flags ^ O_NONBLOCK) < 0)
  {
    perror("create_open():fcntl()");
    close(r->fd);
    r->fd = -1;
    return (-1);
  }

  puts("Done.");

  return (0);
}

int roomba500_init(roomba500_comm_t* r, unsigned char fullcontrol)
{
  unsigned char cmdbuf[1];

  //usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  cmdbuf[0] = ROOMBA_OPCODE_START;
  if (write(r->fd, cmdbuf, 1) < 0)
  {
    perror("create_init():write():");
    return (-1);
  }
  r->mode = ROOMBA_MODE_PASSIVE;

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  if (fullcontrol)
  {
    cmdbuf[0] = ROOMBA_OPCODE_FULL;
    if (write(r->fd, cmdbuf, 1) < 0)
    {
      perror("create_init():write():");
      return (-1);
    }
    r->mode = ROOMBA_MODE_FULL;
  }
  else
  {
    cmdbuf[0] = ROOMBA_OPCODE_SAFE;
    if (write(r->fd, cmdbuf, 1) < 0)
    {
      perror("create_init():write():");
      return (-1);
    }
    r->mode = ROOMBA_MODE_SAFE;

  }

  usleep(100000);

  return (0);
}

int roomba500_close(roomba500_comm_t* r)
{
  //unsigned char cmdbuf[1];

  roomba500_set_speeds(r, 0.0, 0.0);

  usleep(ROOMBA_DELAY_MODECHANGE_MS * 1e3);

  if (close(r->fd) < 0)
  {
    perror("create_close():close():");
    return (-1);
  }
  else
    return (0);
}

int roomba500_set_speeds(roomba500_comm_t* r, double tv, double rv)
{
  unsigned char cmdbuf[5];
  int16_t tv_mm, rad_mm;

  //printf("tv: %.3lf rv: %.3lf\n_", tv, rv);

  tv_mm = (int16_t)rint(tv * 1e3);
  tv_mm = MAX(tv_mm, -ROOMBA_TVEL_MAX_MM_S);
  tv_mm = MIN(tv_mm, ROOMBA_TVEL_MAX_MM_S);

  if (rv == 0)
  {
    // Special case: drive straight
    rad_mm = 0x8000;
  }
  else if (fabs(tv / rv) < 0.24) // assume we want to turn in place when rv is large compared to tv
  {
    // Special cases: turn in place
    if (rv > 0)
      rad_mm = 1;
    else
      rad_mm = -1;

    tv_mm = (int16_t)rint(fabs(rv) * ROOMBA_TVEL_MAX_MM_S * ((1.0 - (fabs(tv / rv) / 0.24)) * 0.5 + 0.5)); // transition more smoothly

    //tv_mm = (int16_t)rint(ROOMBA_AXLE_LENGTH * fabs(rv) * 1e3);
  }
  else
  {
    // General case: convert rv to turn radius
    rad_mm = (int16_t)rint(tv_mm / rv);
    // The robot seems to turn very slowly with the above
    rad_mm /= 2;
    //printf("real rad_mm: %d\n_", rad_mm);
    rad_mm = MAX(rad_mm, -ROOMBA_RADIUS_MAX_MM);
    rad_mm = MIN(rad_mm, ROOMBA_RADIUS_MAX_MM);
  }

  //printf("tv_mm: %d rad_mm: %d\n_", tv_mm, rad_mm);

  cmdbuf[0] = ROOMBA_OPCODE_DRIVE;
  cmdbuf[1] = (unsigned char)(tv_mm >> 8);
  cmdbuf[2] = (unsigned char)(tv_mm & 0xFF);
  cmdbuf[3] = (unsigned char)(rad_mm >> 8);
  cmdbuf[4] = (unsigned char)(rad_mm & 0xFF);

  //printf("set_speeds: %X %X %X %X %X\n_",
  //cmdbuf[0], cmdbuf[1], cmdbuf[2], cmdbuf[3], cmdbuf[4]);

  if (write(r->fd, cmdbuf, 5) < 0)
  {
    perror("create_set_speeds():write():");
    return (-1);
  }
  else
    return (0);
}

int roomba500_get_sensors(roomba500_comm_t* r, int timeout)
{
  struct pollfd ufd[1];
  unsigned char cmdbuf[2];
  unsigned char databuf[ROOMBA_SENSOR_PACKET_SIZE];
  int retval;
  int numread;
  uint totalnumread;
  //int i;

  cmdbuf[0] = ROOMBA_OPCODE_SENSORS;
  /* Zero to get all sensor data */
  cmdbuf[1] = 100; // 0

  if (write(r->fd, cmdbuf, 2) < 0)
  {
    perror("create_get_sensors():write():");
    return (-1);
  }

  ufd[0].fd = r->fd;
  ufd[0].events = POLLIN;

  totalnumread = 0;
  while (totalnumread < sizeof(databuf))
  {
    retval = poll(ufd, 1, timeout);

    if (retval < 0)
    {
      if (errno == EINTR)
        continue;
      else
      {
        perror("create_get_sensors():poll():");
        return (-1);
      }
    }
    else if (retval == 0)
    {
      printf("create_get_sensors: poll timeout\n");
      return (-1);
    }
    else
    {
      if ((numread = read(r->fd, databuf + totalnumread, sizeof(databuf) - totalnumread)) < 0)
      {
        perror("create_get_sensors():read()");
        return (-1);
      }
      else
      {
        totalnumread += numread;
        /*printf("read %d bytes; buffer so far:\n_", numread);
         for(i=0;i<totalnumread;i++)
         printf("%x ", databuf[i]);
         puts("");
         */
      }
    }
  }
  roomba500_parse_sensor_packet(r, databuf, (size_t)totalnumread);
  return (0);
}

int roomba500_parse_sensor_packet(roomba500_comm_t* r, unsigned char* buf, size_t buflen)
{
  unsigned char flag;
  int16_t signed_int;
  uint16_t unsigned_int;
  double dist, angle;
  int idx;

  if (buflen != ROOMBA_SENSOR_PACKET_SIZE)
  {
    puts("create_parse_sensor_packet():packet is wrong size");
    return (-1);
  }

  idx = 0;

  /* Bumps, wheeldrops */
  flag = buf[idx++];
  r->bumper_right = (flag >> 0) & 0x01;
  r->bumper_left = (flag >> 1) & 0x01;
  r->wheeldrop_right = (flag >> 2) & 0x01;
  r->wheeldrop_left = (flag >> 3) & 0x01;

  r->wall = buf[idx++] & 0x01;
  r->cliff_left = buf[idx++] & 0x01;
  r->cliff_frontleft = buf[idx++] & 0x01;
  r->cliff_frontright = buf[idx++] & 0x01;
  r->cliff_right = buf[idx++] & 0x01;
  r->virtual_wall = buf[idx++] & 0x01;

  flag = buf[idx++];
  r->overcurrent_sidebrush = (flag >> 0) & 0x01;
  r->overcurrent_vacuum = (flag >> 1) & 0x01;
  r->overcurrent_mainbrush = (flag >> 2) & 0x01;
  r->overcurrent_driveright = (flag >> 3) & 0x01;
  r->overcurrent_driveleft = (flag >> 4) & 0x01;

  // The next two bytes are unused (used to be dirt detector left and right)
  idx += 2;

  r->remote_opcode = buf[idx++];

  flag = buf[idx++];
  r->button_clean = (flag >> 0) & 0x01;
  r->button_spot = (flag >> 1) & 0x01;
  r->button_dock = (flag >> 2) & 0x01;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  dist = signed_int / 1e3;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  angle = DTOR(signed_int);

  /* First-order odometric integration */
  r->oa = NORMALIZE(r->oa + angle);
  r->ox += dist * cos(r->oa);
  r->oy += dist * sin(r->oa);

  r->charging_state = buf[idx++];

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->voltage = unsigned_int / 1e3;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->current = signed_int / 1e3;

  r->temperature = (char)(buf[idx++]);

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->charge = unsigned_int / 1e3;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->capacity = unsigned_int / 1e3;

  // Following additions by Torsten

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->wall_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->cliff_left_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->cliff_frontleft_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->cliff_frontright_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->cliff_right_data = unsigned_int;

  idx += 3; // unused bytes

  flag = buf[idx++];
  r->internal_charger = (flag >> 0) & 0x01;
  r->home_base_charger = (flag >> 1) & 0x01;

  r->open_interface_mode = buf[idx++];
  r->song_number = buf[idx++];
  r->song_playing = buf[idx++] & 0x01;
  r->oi_stream_num_packets = buf[idx++];

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->velocity = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->radius = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->velocity_left = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->velocity_right = signed_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->encoder_counts_left = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->encoder_counts_right = unsigned_int;

  flag = buf[idx++];

  r->light_bump_left = (flag >> 0) & 0x01;
  r->light_bump_frontleft = (flag >> 1) & 0x01;
  r->light_bump_centerleft = (flag >> 2) & 0x01;
  r->light_bump_centerright = (flag >> 3) & 0x01;
  r->light_bump_frontright = (flag >> 4) & 0x01;
  r->light_bump_right = (flag >> 5) & 0x01;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_left_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_frontleft_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_centerleft_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_centerright_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_frontright_data = unsigned_int;

  memcpy(&unsigned_int, buf + idx, 2);
  idx += 2;
  unsigned_int = ntohs(unsigned_int);
  r->light_bump_right_data = unsigned_int;

  r->light_bump_left = (flag >> 0) & 0x01;
  r->light_bump_frontleft = (flag >> 1) & 0x01;
  r->light_bump_centerleft = (flag >> 2) & 0x01;
  r->light_bump_centerright = (flag >> 3) & 0x01;
  r->light_bump_frontright = (flag >> 4) & 0x01;
  r->light_bump_right = (flag >> 5) & 0x01;

  r->ir_opcode_left = buf[idx++];
  r->ir_opcode_right = buf[idx++];

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->left_motor_current = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->right_motor_current = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->main_brush_current = signed_int;

  memcpy(&signed_int, buf + idx, 2);
  idx += 2;
  signed_int = (int16_t)ntohs((uint16_t)signed_int);
  r->side_brush_current = signed_int;

  r->stasis = (buf[idx++] >> 0) & 0x01;

  return (0);
}

void roomba500_print(roomba500_comm_t* r)
{

  printf("mode: %d\n", r->mode);

  printf("position: %.3lf %.3lf %.3lf\n", r->ox, r->oy, r->oa);

  printf("bumpers: l:%d r:%d\n", r->bumper_left, r->bumper_right);

  printf("wall: %d wall_data: %d virtual wall: %d\n", r->wall, r->wall_data, r->virtual_wall);

  printf("wheeldrops: l:%d r:%d\n", r->wheeldrop_left, r->wheeldrop_right);

  printf("cliff: l:%d fl:%d fr:%d r:%d\n", r->cliff_left, r->cliff_frontleft, r->cliff_frontright, r->cliff_right);

  printf("cliff_front_right:%d cliff_right:%d\n", r->cliff_frontright_data, r->cliff_right_data);

  printf("cliff_left:%d cliff_front_right:%d\n", r->cliff_left_data, r->cliff_frontleft_data);

  printf("overcurrent: dl:%d dr:%d mb:%d sb:%d v:%d\n", r->overcurrent_driveleft, r->overcurrent_driveright,
         r->overcurrent_mainbrush, r->overcurrent_sidebrush, r->overcurrent_vacuum);

  printf("remote opcode: %d\n", r->remote_opcode);

  printf("buttons: dock:%d spot:%d clean:%d\n", r->button_dock, r->button_spot, r->button_clean);

  printf("charging state: %d\n", r->charging_state);

  printf("battery: voltage:%.3lf current:%.3lf temp:%.3lf charge:%.3lf capacity:%.3f\n", r->voltage, r->current,
         r->temperature, r->charge, r->capacity);

  printf("home charger: %d internal charger: %d\n", r->home_base_charger, r->internal_charger);

  printf("open_interface_mode: %d\n", r->open_interface_mode);

  printf("song number: %d song playing: %d\n", r->song_number, r->song_playing);

  printf("oi_stream_num_packets: %d\n", r->oi_stream_num_packets);

  printf("velocity: %d radius: %d vel_right: %d vel_left: %d\n", r->velocity, r->radius, r->velocity_right,
         r->velocity_left);

  printf("encoder_count_left: %d encoder_count_right: %d\n", r->encoder_counts_left, r->encoder_counts_right);

  printf("Light Bump left: %d front_left: %d center_left: %d\n", r->light_bump_left, r->light_bump_frontleft,
         r->light_bump_centerleft);
  printf("Light Bump right: %d front_right: %d center_right: %d\n", r->light_bump_right, r->light_bump_frontright,
         r->light_bump_centerright);

  printf("Light Bump left data: %d front_left data: %d center_left data: %d\n", r->light_bump_left_data,
         r->light_bump_frontleft_data, r->light_bump_centerleft_data);
  printf("Light Bump right data: %d front_right data: %d center_right data: %d\n", r->light_bump_right_data,
         r->light_bump_frontright_data, r->light_bump_centerright_data);

  printf("Ir opcode left: %d Ir opcode right: %d\n", r->ir_opcode_left, r->ir_opcode_right);

  printf("Left motor current: %d Right motor current: %d\n", r->left_motor_current, r->right_motor_current);

  printf("Main brush current: %d Side brush current: %d", r->main_brush_current, r->side_brush_current);

  printf("Stasis: %d\n", r->stasis);

}

int roomba500_set_song(roomba500_comm_t* r, unsigned char songNumber, unsigned char songLength, unsigned char *notes,
                       unsigned char *noteLengths)
{
  int size = 2 * songLength + 3;
  unsigned char cmdbuf[size];
  unsigned char i;

  cmdbuf[0] = ROOMBA_OPCODE_SONG;
  cmdbuf[1] = songNumber;
  cmdbuf[2] = songLength;

  for (i = 0; i < songLength; i++)
  {
    cmdbuf[3 + (2 * i)] = notes[i];
    cmdbuf[3 + (2 * i) + 1] = noteLengths[i];
  }

  if (write(r->fd, cmdbuf, size) < 0)
  {
    perror("create_set_song():write():");
    return (-1);
  }
  else
    return (0);
}

int roomba500_play_song(roomba500_comm_t *r, unsigned char songNumber)
{
  unsigned char cmdbuf[2];

  cmdbuf[0] = ROOMBA_OPCODE_PLAY;
  cmdbuf[1] = songNumber;

  if (write(r->fd, cmdbuf, 2) < 0)
  {
    perror("create_set_song():write():");
    return (-1);
  }
  else
    return (0);
}

int roomba500_vacuum(roomba500_comm_t *r, int state)
{
  unsigned char cmdbuf[2];

  cmdbuf[0] = ROOMBA_OPCODE_MOTORS;
  cmdbuf[1] = state;

  if (write(r->fd, cmdbuf, 2) < 0)
  {
    perror("create_vacuum():write():");
    return -1;
  }

  return 0;
}

// Set LEDs
// set <dirt_detect>, <spot>, <dock>, <status> to 1 to light the corresponding leds
// the main led_ is lit iff <power_intensity> is greater than 0
// color of main led_: 0 == green, ~20 == orange, >100 == red

int roomba500_set_leds(roomba500_comm_t *r, uint8_t dirt_detect, uint8_t dock, uint8_t power_color,
                       uint8_t power_intensity, uint8_t spot, uint8_t status)
{
  unsigned char cmdbuf[5];
  cmdbuf[0] = ROOMBA_OPCODE_LEDS;
  cmdbuf[1] = dirt_detect | spot << 1 | dock << 2 | status << 3;
  cmdbuf[2] = power_color;
  cmdbuf[3] = power_intensity;

  //printf("Set LEDS[%d][%d][%d]\n_",cmdbuf[1], cmdbuf[2], cmdbuf[3]);
  if (write(r->fd, cmdbuf, 4) < 0)
  {
    perror("create_set_leds():write():");
    return -1;
  }

  return 0;
}

int roomba500_run_demo(roomba500_comm_t *r, uint8_t num)
{
  unsigned char cmdbuf[2];
  cmdbuf[0] = ROOMBA_OPCODE_DEMO;
  cmdbuf[1] = num;

  if (write(r->fd, cmdbuf, 2) < 0)
  {
    perror("create_run_demo():write():");
    return -1;
  }
  return 0;
}

