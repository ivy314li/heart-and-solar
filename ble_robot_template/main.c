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

#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "buckler.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);
int x_pos_readable = 0;
int y_pos_readable = 0;
int angle_readable = 0;
bool ble_connected = false;

// global variables
KobukiSensors_t sensors = {0};

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x6969, // TODO: replace with your lab bench number
        .adv_name          = "HEARTSOLAR", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

// service
static simple_ble_service_t angle_service = {{
    .uuid128 = {0xca,0xc4,0x87,0x0d,0x42,0x84,0xff,0xA9,
                0x59,0x4D,0x5e,0xf6,0xa0,0xed,0x07,0x46}
}};

// characteristics
static simple_ble_char_t x_pos_char = {.uuid16 = 0xa4ca};
static int xpos = 32;
static simple_ble_char_t y_pos_char = {.uuid16 = 0xb4cb};
static int ypos = 8;
static simple_ble_char_t angle_char = {.uuid16 = 0xc4cb};
static int angle = -1;
static simple_ble_char_t update_char = {.uuid16 = 0xd4cd};
static bool updated = false; //has a new update been written by bluetooth?

// main application state
simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    // receive measurements
    // if (simple_ble_is_char_event(p_ble_evt, &update_char)) {
    //   printf("position was updated\n");
    //   display_write("got bluetooth update", DISPLAY_LINE_0);
    //   if (updated) {
    //     printf("updating variables...\n");
    //     y_pos_readable = ypos;
    //     x_pos_readable = xpos;
    //     angle_readable = angle;
    //     //index_readable = index;
    //     // updated = false;
    //   } else {
    //     printf("no bluetooth update\n");
    //   }
    // }
    y_pos_readable = ypos;
    x_pos_readable = xpos;
    angle_readable = angle;
    updated = true;
}

// extern void ble_evt_connected(ble_evt_t const* p_ble_evt) {
//     display_write("BLE CONNECTED", DISPLAY_LINE_0);
//     ble_connected = true;
//     nrf_delay_ms(1000);
// }

// extern void ble_evt_disconnected(ble_evt_t const* p_ble_evt) {
//     display_write("BLE DISCONNECTED :(", DISPLAY_LINE_0);
//     ble_connected = false;
//     nrf_delay_ms(1000);
// }

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
const uint8_t POWER_CHANNEL = 0;

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

static position get_position() {
  // TODO: use ble to get position
  // currently returns one stupid position

  position curr = {0,0,0};
  // if (!ble_connected) {
  //   return curr;
  // }
  int timeout = 3;

  int timer = 0;
  // while(!updated) {
  //     nrf_delay_ms(1000);
  //     printf("reached\n");
  //     timer += 1;
  //     display_write("waiting 4 update...", DISPLAY_LINE_1);
  //     if (timer >= timeout) {
  //       return curr;
  //     }
  // }

  curr.x = (float) x_pos_readable / 100;
  curr.y = (float) y_pos_readable /100;
  curr.theta = (float) angle_readable;
  updated = false;
  return curr;

  // position from;
  // from.x = .75;
  // from.y = .75;
  // from.theta = 1.57;
  // return from;
}

// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

// sample a particular analog channel in blocking mode
nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
}

float adc_to_voltage(nrf_saadc_value_t adc_code) {
  return (float) adc_code * 3.6/4096;
}

float voltage_to_power(float voltage) {
  // TODO: finish this
  return 0.0;
}

float fake_measurements[] = {0.5, 1.2, 3.3, 0.1, 0.6, 2.3, 1.1, 1.2, 0.4, 5.2};
int k = 0;
float get_measurement() {
  return adc_to_volage(sample_value(POWER_CHANNEL));
  //return fake_measurements[k++];
}

/* Return a new position with the x and y coordinates of position P floored */
position floor_position(position *p) {
  position flr_pos = {floor(p->x), floor(p->y), p->theta};
  return flr_pos;
}

/* Given a position, find the center of the tile. P must be a floored position. */
position find_center(position *p) {
  position center;
  center.x = p->x + SIDE_LEN/2;
  center.y = p->y + SIDE_LEN/2;
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
  float target_angle_from_vert = atan2(to->x - from->x, from->y - to->y) * 180 / M_PI;
  result.turn_angle = target_angle_from_vert - from->theta;
  return result;
}

const int WINDOW_SIZE = 10;
float measurements[10]; // The last WINDOW_SIZE measurements
int write_index = 0;

/* The position that produces the max power as well as its measurement. */
struct max_power_position {
  position p;
  float measurement;
};

/* Return the average of the MEASUREMENTS buffer. */
float get_averaged_measurement() {
  float total = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    total += measurements[i];
  }
  return total/WINDOW_SIZE;
}

/* Write a measurement into the MEASUREMENTS buffer. Replace the oldest measurement. */
void log_measurement(float m) {
  measurements[write_index] = m;
  write_index++;
  if (write_index >= WINDOW_SIZE) {
    write_index = 0;
  }
}

/* Update the max power position MPP. */
void update_mpp(struct max_power_position *mpp) {
  float cur_measurement = get_averaged_measurement();
  position cur_position = get_position();
  if (cur_measurement > mpp->measurement) {
    mpp->measurement = cur_measurement;
    mpp->p = cur_position;
  }
}

const int grid_size = 3; //Number of tiles in a row on the grid.

uint16_t previous_encoder;
uint16_t current_encoder;
float degrees = 0;
float turn_angle = 0;
float drive_distance = 0;
int step_counter = 0; // The number of steps the robot has taken while traversing.
bool turn_right = false; // Whether the robot should turn right or left while traversing the grid.
bool switch_directions;
dist_angle directions;


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // initialize analog to digital converter
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // initialize analog inputs
  // configure with 0 as input pin for now
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
  channel_config.gain = NRF_SAADC_GAIN1_6; // input gain of 1/6 Volts/Volt, multiply incoming signal by (1/6)
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6 Volt reference, input after gain can be 0 to 0.6 Volts

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = NRF_SAADC_INPUT_AIN2;
  error_code = nrfx_saadc_channel_init(POWER_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  struct max_power_position mpp;
  float distance = 0;
  error_code = NRF_SUCCESS;
  // position impos;
  // impos.x = 0.5;
  // impos.y = 0.75;
  // impos.theta = 0;
  // mpp.p = impos;
  // mpp.measurement = 0.0;
  int timer;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
  simple_ble_add_service(&angle_service);
  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
      sizeof(angle), (uint8_t*)&angle,
      &angle_service, &angle_char);
  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
      sizeof(xpos), (uint8_t*)&xpos,
      &angle_service, &x_pos_char);
  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
      sizeof(ypos), (uint8_t*)&ypos,
      &angle_service, &y_pos_char);
  simple_ble_add_characteristic(1, 1, 0, 0, // read, write, notify, vlen
      sizeof(updated), (uint8_t*)&updated,
      &angle_service, &update_char);

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
  while (1) {
    float v = adc_to_voltage(sample_value(POWER_CHANNEL));
    char buf[16];
    snprintf(buf, 16, "%f", v);
    display_write(buf, DISPLAY_LINE_0);
  }
  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    current_encoder = sensors.leftWheelEncoder;
    //int status = kobukiSensorPoll(&sensors);
    log_measurement(get_measurement());

    //set initial mpp values
    //nrf_delay_ms(1);
    // TODO: complete state machine
    switch(state) {
      case OFF: {
        //print_state(state);

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
        //nrf_delay_ms(2000);
        printf("traverse state\n");
        //position c = get_position();
       	timer = 0;
        while (timer < 20) {
        	nrf_delay_ms(100);
        	kobukiSensorPoll(&sensors);
        	timer++;
        }
        update_mpp(&mpp);
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
        //update_mpp(&mpp);
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
          kobukiDriveDirect(85,84);
          state = DRIVE_STEP;
          char buf[16];
          snprintf(buf, 16, "%f", distance);
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
          kobukiDriveDirect(46,-50);
          state = TURN_90;
        } else {
          kobukiDriveDirect(-50,46);
          state = TURN_90;
        }
        break;
      }

      case NAV: { // navigate to the best position
        print_state(state);
        printf("nav state\n");
  
        if (is_button_pressed(&sensors)) {
          state = OFF;
        }
        else {
        	timer = 0;
        	while (timer < 40) {
        		nrf_delay_ms(100);
        		kobukiSensorPoll(&sensors);
        		timer++;
        	}

        	position cur_position = get_position();
	        char buf[16];
	        snprintf(buf, 16, "%f", cur_position.x);
	        display_write(buf, DISPLAY_LINE_0);
	        snprintf(buf, 16, "Angle: %f", cur_position.theta);
			display_write(buf, DISPLAY_LINE_1);
			// state = OFF;
	  //       break;
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
        }
        //nrf_delay_ms(2000);
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
        } else if (fabs(degrees) >= fabs(turn_angle)) { //fabs(turn_angle*180/3.14)
          state = DRIVE;
          previous_encoder = sensors.leftWheelEncoder;
          distance = 0.0;
          drive_distance = directions.dist;
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(75, 75);
          //nrf_delay_ms(100);
          //break;
        } else if (turn_right) {
          kobukiDriveDirect(46,-50);
          state = TURN;
        } else {
          kobukiDriveDirect(-50,46);
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
          display_write(buf, DISPLAY_LINE_1);

          //display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
  }
}
}
