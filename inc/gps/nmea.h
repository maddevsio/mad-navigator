#ifndef NMEA_H
#define NMEA_H

/*
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include "time/kov_time.h"
#define NMEA_MAX_LENGTH 82

enum nmea_sentence_id {
  NMEA_INVALID = -1,
  NMEA_UNKNOWN = 0,
  NMEA_SENTENCE_RMC,
  NMEA_SENTENCE_GGA,
  NMEA_SENTENCE_GSA,
  NMEA_SENTENCE_GLL,
  NMEA_SENTENCE_GST,
  NMEA_SENTENCE_GSV,
  NMEA_SENTENCE_VTG,
  NMEA_SENTENCE_ZDA,
};

struct nmea_float {
  int_least32_t value;
  int_least32_t scale;
};

struct nmea_sentence_rmc {
  kov_time_t time;
  kov_date_t date;
  bool valid;
  struct nmea_float latitude;
  struct nmea_float longitude;
  struct nmea_float speed;
  struct nmea_float course;
  struct nmea_float variation;
};

struct nmea_sentence_gga {
  kov_time_t time;
  struct nmea_float latitude;
  struct nmea_float longitude;
  int fix_quality;
  int satellites_tracked;
  struct nmea_float hdop;
  struct nmea_float altitude; char altitude_units;
  struct nmea_float height; char height_units;
  struct nmea_float dgps_age;
};

enum nmea_gll_status {
  NMEA_GLL_STATUS_DATA_VALID = 'A',
  NMEA_GLL_STATUS_DATA_NOT_VALID = 'V',
};

// FAA mode added to some fields in NMEA 2.3.
enum nmea_faa_mode {
  NMEA_FAA_MODE_AUTONOMOUS = 'A',
  NMEA_FAA_MODE_DIFFERENTIAL = 'D',
  NMEA_FAA_MODE_ESTIMATED = 'E',
  NMEA_FAA_MODE_MANUAL = 'M',
  NMEA_FAA_MODE_SIMULATED = 'S',
  NMEA_FAA_MODE_NOT_VALID = 'N',
  NMEA_FAA_MODE_PRECISE = 'P',
};

struct nmea_sentence_gll {
  struct nmea_float latitude;
  struct nmea_float longitude;
  kov_time_t time;
  char status;
  char mode;
};

struct nmea_sentence_gst {
  kov_time_t time;
  struct nmea_float rms_deviation;
  struct nmea_float semi_major_deviation;
  struct nmea_float semi_minor_deviation;
  struct nmea_float semi_major_orientation;
  struct nmea_float latitude_error_deviation;
  struct nmea_float longitude_error_deviation;
  struct nmea_float altitude_error_deviation;
};

enum nmea_gsa_mode {
  NMEA_GPGSA_MODE_AUTO = 'A',
  NMEA_GPGSA_MODE_FORCED = 'M',
};

enum nmea_gsa_fix_type {
  NMEA_GPGSA_FIX_NONE = 1,
  NMEA_GPGSA_FIX_2D = 2,
  NMEA_GPGSA_FIX_3D = 3,
};

struct nmea_sentence_gsa {
  char mode;
  int fix_type;
  int sats[12];
  struct nmea_float pdop;
  struct nmea_float hdop;
  struct nmea_float vdop;
};

struct nmea_sat_info {
  int nr;
  int elevation;
  int azimuth;
  int snr;
};

struct nmea_sentence_gsv {
  int total_msgs;
  int msg_nr;
  int total_sats;
  struct nmea_sat_info sats[4];
};

struct nmea_sentence_vtg {
  struct nmea_float true_track_degrees;
  struct nmea_float magnetic_track_degrees;
  struct nmea_float speed_knots;
  struct nmea_float speed_kph;
  enum nmea_faa_mode faa_mode;
};

struct nmea_sentence_zda {
  kov_time_t time;
  kov_date_t date;
  int hour_offset;
  int minute_offset;
};

/**
 * Calculate raw sentence checksum. Does not check sentence integrity.
 */
uint8_t nmea_checksum(const char *sentence);

/**
 * Check sentence validity and checksum. Returns true for valid sentences.
 */
bool nmea_check(const char *sentence, bool strict);

/**
 * Determine talker identifier.
 */
bool nmea_talker_id(char talker[3], const char *sentence);

/**
 * Determine sentence identifier.
 */
enum nmea_sentence_id nmea_sentence_id(const char *sentence, bool strict);

/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - direction, returned as 1/-1, default 0 (int *)
 * f - fractional, returned as value + scale (int *, int *)
 * i - decimal, default zero (int *)
 * s - string (char *)
 * t - talker identifier and type (char *)
 * T - date/time stamp (int *, int *, int *)
 * Returns true on success. See library source code for details.
 */
bool nmea_scan(const char *sentence, const char *format, ...);

/*
 * Parse a specific type of sentence. Return true on success.
 */
bool nmea_parse_rmc(struct nmea_sentence_rmc *frame, const char *sentence);
bool nmea_parse_gga(struct nmea_sentence_gga *frame, const char *sentence);
bool nmea_parse_gsa(struct nmea_sentence_gsa *frame, const char *sentence);
bool nmea_parse_gll(struct nmea_sentence_gll *frame, const char *sentence);
bool nmea_parse_gst(struct nmea_sentence_gst *frame, const char *sentence);
bool nmea_parse_gsv(struct nmea_sentence_gsv *frame, const char *sentence);
bool nmea_parse_vtg(struct nmea_sentence_vtg *frame, const char *sentence);
bool nmea_parse_zda(struct nmea_sentence_zda *frame, const char *sentence);

/**
 * Convert GPS UTC date/time representation to a UNIX timestamp.
 */
kov_timestamp_t nmea_gettime(const kov_date_t *date, const kov_time_t *time_);


/**
 * Rescale a fixed-point value to a different scale. Rounds towards zero.
 */
int_least32_t nmea_rescale(struct nmea_float *f, int_least32_t new_scale);

/**
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
float nmea_tofloat(const struct nmea_float *f);

/**
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
float nmea_tocoord(const struct nmea_float *f);

#endif // NMEA_H
