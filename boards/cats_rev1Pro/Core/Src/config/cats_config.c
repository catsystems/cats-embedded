//
// Created by stoja on 21.12.20.
//

#include "config/cats_config.h"
#include "drivers/w25q256.h"
#include "util/log.h"
#include "drivers/eeprom_emul.h"

typedef struct {
  /* Last sector where task_recorder wrote the data; The next sector will be
   * first checked if it's empty and if so, the next flight recorder
   * log will be recorded starting from that sector */
  uint16_t last_recorded_sector;
  /* Total number of currently logged flights on the flash chip. This value
   * represents the length of the array stored in sector #1 that holds the
   * starting sectors of separate flight logs. */
  uint16_t num_recorded_flights;
  /* TODO: don't create a static array here */
  uint16_t last_sectors_of_flight_recordings[32];

  flight_fsm_e last_fsm_state[32];
  float max_altitude[32];
  float max_velocity[32];
  float max_acceleration[32];
} cats_status_t;

const cats_config_u DEFAULT_CONFIG = {
    .config.boot_state = CATS_FLIGHT,
    .config.control_settings.main_altitude = 150,
    .config.control_settings.liftoff_acc_threshold = 1500,
    .config.timers[0].duration = 1000,
    .config.timers[0].start_event = 0,
    .config.timers[0].end_event = 0,
    .config.timers[1].duration = 1000,
    .config.timers[1].start_event = 0,
    .config.timers[1].end_event = 0,
    .config.timers[2].duration = 1000,
    .config.timers[2].start_event = 0,
    .config.timers[2].end_event = 0,
    .config.timers[3].duration = 1000,
    .config.timers[3].start_event = 0,
    .config.timers[3].end_event = 0,
};

cats_config_u global_cats_config = {};
cats_status_t global_cats_status = {};

const uint32_t CATS_STATUS_SECTOR = 1;

/** cats config initialization **/

void cc_init() {
  HAL_FLASH_Unlock();
  EE_Status ee_status = EE_Init(EE_FORCED_ERASE);
  if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
}
void cc_defaults() {
  memcpy(&global_cats_config, &DEFAULT_CONFIG, sizeof(global_cats_config));
}

/** persistence functions **/

void cc_load() {
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    EE_ReadVariable32bits(i + 1, &global_cats_config.config_array[i]);
  }
}

void cc_save() {
  for (int i = 0; i < sizeof(cats_config_t) / sizeof(uint32_t); i++) {
    uint32_t tmp;
    EE_ReadVariable32bits(i + 1, &tmp);
    if (tmp != global_cats_config.config_array[i]) {
      EE_Status ee_status = EE_WriteVariable32bits(i + 1, global_cats_config.config_array[i]);
      if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) EE_CleanUp();
    }
  }
}

/** debug functions **/

void cc_print() {
  static const char *BOOT_STATE_STRING[] = {"CATS_INVALID", "CATS_IDLE", "CATS_CONFIG",
                                            "CATS_TIMER",   "CATS_DROP", "CATS_FLIGHT"};
  log_info("Boot State: %s", BOOT_STATE_STRING[global_cats_config.config.boot_state]);
}

/** cats status initialization **/

void cs_init(uint16_t last_recorded_sector, uint16_t num_recorded_flights) {
  log_info("Clearing cats_status");
  cs_clear();
  global_cats_status.last_recorded_sector = last_recorded_sector;
  global_cats_status.num_recorded_flights = num_recorded_flights;
}

void cs_clear() { memset(&global_cats_status, 0, sizeof(global_cats_status)); }

/** accessor functions **/

uint32_t cs_get_last_recorded_sector() { return global_cats_status.last_recorded_sector; }
void cs_set_last_recorded_sector(uint32_t last_recorded_sector) {
  global_cats_status.last_recorded_sector = last_recorded_sector;
  /* Increment the last element of last_sectors_of_flight_recordings */
  /* 32 is good here because we are reducing it by 1 when indexing */
  if (global_cats_status.num_recorded_flights > 0 && global_cats_status.num_recorded_flights <= 32) {
    global_cats_status.last_sectors_of_flight_recordings[global_cats_status.num_recorded_flights - 1] =
        last_recorded_sector;
  }
}

uint32_t cs_get_last_sector_of_flight(uint16_t flight_idx) {
  if (flight_idx < 32) {
    return global_cats_status.last_sectors_of_flight_recordings[flight_idx];
  } else
    return 0;
}

uint16_t cs_get_num_recorded_flights() { return global_cats_status.num_recorded_flights; }
void cs_set_num_recorded_flights(uint16_t num_recorded_flights) {
  global_cats_status.num_recorded_flights = num_recorded_flights;
}

void cs_set_max_altitude(float altitude) {
  global_cats_status.max_altitude[global_cats_status.num_recorded_flights - 1] = altitude;
}

void cs_set_max_velocity(float velocity) {
  global_cats_status.max_velocity[global_cats_status.num_recorded_flights - 1] = velocity;
}

void cs_set_max_acceleration(float acceleration) {
  global_cats_status.max_acceleration[global_cats_status.num_recorded_flights - 1] = acceleration;
}

float cs_get_max_acceleration(uint32_t flight) {
  if (flight <= global_cats_status.num_recorded_flights)
    return global_cats_status.max_acceleration[flight];
  else
    return 0;
}

float cs_get_max_altitude(uint32_t flight) {
  if (flight <= global_cats_status.num_recorded_flights)
    return global_cats_status.max_altitude[flight];
  else
    return 0;
}

float cs_get_max_velocity(uint32_t flight) {
  if (flight <= global_cats_status.num_recorded_flights)
    return global_cats_status.max_velocity[flight];
  else
    return 0;
}

void cs_set_flight_phase(flight_fsm_e state) {
  global_cats_status.last_fsm_state[global_cats_status.num_recorded_flights - 1] = state;
}

/** persistence functions **/

void cs_load() {
  /* TODO: global_cats_status can't be larger than sector size */
  w25q_read_buffer((uint8_t *)(&global_cats_status), CATS_STATUS_SECTOR * w25q.sector_size, sizeof(global_cats_status));
}

void cs_save() {
  /* erase sector before writing to it */
  w25q_sector_erase(CATS_STATUS_SECTOR * w25q.sector_size);
  /* TODO: global_cats_status can't be larger than sector size */
  w25q_write_buffer((uint8_t *)(&global_cats_status), CATS_STATUS_SECTOR * w25q.sector_size, sizeof(global_cats_status));
}

/** debug functions **/

void cs_print() {
  log_info("Config: Last recorded sector: %u; Number of recorded flights: %u", global_cats_status.last_recorded_sector,
           global_cats_status.num_recorded_flights);
}
