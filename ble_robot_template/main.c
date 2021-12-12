// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"
#include "simple_ble.h"

#include "states.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// global variables
KobukiSensors_t sensors = {0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0000, // TODO: replace with your lab bench number
        .adv_name          = "KOBUKI", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

//4607eda0-f65e-4d59-a9ff-84420d87a4ca
static simple_ble_service_t robot_service = {{
    .uuid128 = {0xca,0xa4,0x87,0x0d,0x42,0x84,0xff,0xA9,
                0x59,0x4D,0x5e,0xf6,0xa0,0xed,0x07,0x46}
}};

// TODO: Declare characteristics and variables for your service

simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    // TODO: logic for each characteristic and related state changes
}

void print_state(states current_state){
  switch(current_state){
      case OFF: {
        display_write("OFF", DISPLAY_LINE_0);
        break;
      }
      case NAV: {
        display_write("NAV", DISPLAY_LINE_0);
        //nrf_delay_ms(1000);
        break;
      }
      case TURN_90: {
        display_write("TURN_90", DISPLAY_LINE_0);
        break;
      }
      case TRAVERSE: {

        display_write("TRAVERSE", DISPLAY_LINE_0);
        //nrf_delay_ms(1000); this will literally breka everything do not uncomment
        break;
      }
      case DRIVE_STEP: {
        display_write("DRIVE_STEP", DISPLAY_LINE_0);
        break;
      }
      case DRIVE: {
        display_write("DRIVE", DISPLAY_LINE_0);
        break;
      }
      case TURN: {
        display_write("TURN", DISPLAY_LINE_0);
        break;
      }
    }
}

const float SIDE_LEN = 0.30; // Length of a tile in meters

/* Stores the x and y positions as well as the angle of the robot */
typedef struct position_tuple {
  float x;
  float y;
  float theta;
} position;

/* Stores a distance and an angle */
typedef struct dist_angle_tuple {
  float dist;
  float turn_angle;
} dist_angle;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  // yo i hope this is in meters lol
  const float CONVERSION = 0.00065;

  float result = 0.0;
  if (current_encoder >= previous_encoder) {
    result = (float)current_encoder - (float)previous_encoder;
  } else {
    result = (float)current_encoder + (0xFFFF - (float)previous_encoder);
  }
  result = result * CONVERSION;
  if (result > 0.10) {
      display_write("NOPE", DISPLAY_LINE_1);
      // filter out unrealistic distance traveled
      return 0.0;
  }
  char buf[16];
  snprintf(buf, 16, "%d", result);
  display_write(buf, DISPLAY_LINE_1);
  //display_write("NOPE", DISPLAY_LINE_1);
  return result;
}

position get_position() {
  // TODO: use ble to get position
  // currently returns one stupid position
  position from;
  from.x = .75;
  from.y = .75;
  from.theta = 1.57;
  return from;
}

float get_measurement() {
  // TODO: get measurement from light sensor or solar converter
  return 0.0;
}

/* Return a new position with the x and y coordinates of position P floored */
position floor_position(position *p) {
  position flr_pos = {floor(p->x), floor(p->y), p->theta};
  return flr_pos;
}

/* Given a position, find the center of the tile. P must be a floored position. */
position find_center(position *p) {
  position center;
  center.x = p->x + SIDE_LEN;
  center.y = p->y + SIDE_LEN;
  center.theta = p->theta;
  return center;
}

/* Return the distance and turn angle needed to reach the center of position TO from position FROM. WIP DON'T USE THIS.*/
// dist_angle navigate_center(position *from, position *to) {
//   position center_from = find_center(&floor_position(from));
//   position center_to = find_center(&floor_position(to));
//   dist_angle result;
//   result.distance = sqrt(pow(center_from.x - center_to.x, 2) + pow(center_from.y - center_to.y, 2));
//   result.angle = to->angle - from->angle;
//   return result;
// }

/* Return the distance and turn angle needed to reach the center of position TO from position FROM. */
dist_angle navigate(position *from, position *to) {
  // printf("from x %f\n", from->x);
  // printf("from y %f\n", from->y);
  // printf("from angle %f\n", from->theta);
  // printf("to x %f\n", to->x);
  // printf("ot y %f\n", to->y);
  // printf("from angle %f\n", to->theta);
  dist_angle result;
  result.dist = sqrt(pow(from->x - to->x, 2) + pow(from->y - to->y, 2));
  //float target_angle_from_vert = atan2(to->y - from->y, to->x - from->x);
  float target_angle_from_vert = atan2(to->x - from->x, from->y - to->y);
  result.turn_angle = target_angle_from_vert - from->theta;
  return result;
}



struct max_power_position {
  position p;
  float measurement;
};

const int grid_size = 3; //Number of tiles in a row on the grid.

uint16_t previous_encoder;
uint16_t current_encoder;
float degrees = 0;
float turn_angle = 0;
float drive_distance = 0;
static struct max_power_position mpp;
int step_counter = 0; // The number of steps the robot has taken while traversing.
bool turn_right = false; // Whether the robot should turn right or left while traversing the grid.
float cur_measurement;
position cur_position;
bool switch_directions;
dist_angle directions;

int main(void) {
  float distance = 0;
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&robot_service);

  // TODO: Register your characteristics

  // Start Advertising
  simple_ble_adv_only_name();

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  states state = OFF;

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    current_encoder = sensors.leftWheelEncoder;
    //int status = kobukiSensorPoll(&sensors);

    //set initial mpp values
    position impos;
    impos.x = .15;
    impos.y = .15;
    impos.theta = 1.57;
    mpp.p = impos;
    mpp.measurement = 1.0;

    // TODO: complete state machine
    switch(state) {
      case OFF: {
        print_state(state);

        // transition logic
        if (is_button_pressed(&sensors)) {
        //if (true) { //kms
          //state = NEWSTATE;
          state = TRAVERSE;
          previous_encoder = sensors.leftWheelEncoder;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }
        break; // each case needs to end with break!
      }
      case TRAVERSE: {
        print_state(state);
        cur_measurement = get_measurement();
        //cur_position = get_position();
        if (cur_measurement > mpp.measurement) {
          mpp.measurement = cur_measurement;
          mpp.p = cur_position;
        }

        if (is_button_pressed(&sensors)) {
          state = OFF;
        }

        else if (step_counter == grid_size*grid_size - 1) { //finished traversing
          state = NAV;
          kobukiDriveDirect(0, 0);
        } else if ((step_counter + 1) % grid_size == 0) { // Detects when we reach the end of a row
          degrees = 0;
          turn_right = !turn_right;
          switch_directions = true;
          lsm9ds1_start_gyro_integration();
          state = TURN_90;
        } else {
          distance = 0;
          previous_encoder = sensors.leftWheelEncoder;
          state = DRIVE_STEP;
          kobukiDriveDirect(75,75);
        }
        break;
      }
      case DRIVE_STEP: {
        print_state(state);
        cur_measurement = get_measurement();
        //cur_position = get_position();
        if (cur_measurement > mpp.measurement) {
          mpp.measurement = cur_measurement;
          mpp.p = cur_position;
        }
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (distance >= SIDE_LEN) {
          if (switch_directions) {
            step_counter++;
            state = TURN_90;
            degrees = 0;
            switch_directions = false;
            lsm9ds1_start_gyro_integration();
          } else {
            step_counter++;
            previous_encoder = sensors.leftWheelEncoder;
            kobukiDriveDirect(0, 0);
            state = TRAVERSE;
          }
        } else {
          current_encoder = sensors.leftWheelEncoder;
          distance += measure_distance(current_encoder, previous_encoder);
          previous_encoder = current_encoder;
          kobukiDriveDirect(75,75);
          state = DRIVE_STEP;
          char buf[16];
          snprintf(buf, 16, "%d", step_counter);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
      case TURN_90: {
        print_state(state);
        degrees = (lsm9ds1_read_gyro_integration()).z_axis;
        char buf[16];
        snprintf(buf, 16, "%f", degrees);
        display_write(buf, DISPLAY_LINE_1);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          break;
        } else if (fabs(degrees) >= 90) {
          if (switch_directions) {
            state = DRIVE_STEP;
            distance = 0;
            previous_encoder = sensors.leftWheelEncoder;
            lsm9ds1_stop_gyro_integration();
            kobukiDriveDirect(75, 75);
          } else {
            state = TRAVERSE;
            previous_encoder = sensors.leftWheelEncoder;
            lsm9ds1_stop_gyro_integration();
            degrees = 0;
            kobukiDriveDirect(0, 0);
          }
        } else if (turn_right) {
          kobukiDriveDirect(50,-50);
          state = TURN_90;
        } else {
          kobukiDriveDirect(-50,50);
          state = TURN_90;
        }
        break;
      }

      case NAV: { // navigate to the best position
        print_state(state);
        printf("nav state\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
          break;
        }
        cur_position = get_position();
        //printf("nav state1\n");
        directions = navigate(&cur_position, &mpp.p);
        //printf("nav state2\n");
        if (directions.turn_angle < 0) {
          turn_right = false;
        } else {
          turn_right = true;
        }
        turn_angle = directions.turn_angle;
        //printf("nav state3\n");
        degrees = 0;
        lsm9ds1_start_gyro_integration();
        //printf("nav state4\n");
        state = TURN;
        break;
      }

      case TURN: {
        print_state(state);
        char buf[16];
        snprintf(buf, 16, "%f", turn_angle);
        display_write(buf, DISPLAY_LINE_1);
        degrees = (lsm9ds1_read_gyro_integration()).z_axis;
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          break;
        } else if (fabs(degrees) >= fabs(turn_angle*180/3.14)) { //fabs(turn_angle*180/3.14)
          state = DRIVE;
          previous_encoder = sensors.leftWheelEncoder;
          distance = 0.0;
          drive_distance = directions.dist;
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(75, 75);
          nrf_delay_ms(100);
          //break;
        } else if (turn_right) {
          kobukiDriveDirect(50,-50);
          state = TURN;
        } else {
          kobukiDriveDirect(-50,50);
          state = TURN;
        }
        break;
      }

      case DRIVE: {
        print_state(state);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          break;
        } else if (distance >= drive_distance) {
          kobukiDriveDirect(0, 0);
          state = OFF;
        } else {


          current_encoder = sensors.leftWheelEncoder;
          printf("%d\n", current_encoder);
          printf("%d\n\n", previous_encoder);
          distance += measure_distance(current_encoder, previous_encoder);
          previous_encoder = current_encoder;
          kobukiDriveDirect(75,75);
          state = DRIVE;

          char buf[16];
          snprintf(buf, 16, "%f", distance);
          nrf_delay_ms(500);

          //display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
    }
  }
}
