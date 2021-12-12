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
int x_pos_readable = 0;
int y_pos_readable = 0;
//int index_readable = 0;
int angle_readable = 0;
bool ble_connected = false;

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x6969, //TODO: replace with your lab bench number
        .adv_name          = "HEARTSOLAR", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

//4607eda0-f65e-4d59-a9ff-84420d87c4ca

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
    if (simple_ble_is_char_event(p_ble_evt, &update_char)) {
      printf("position was updated\n");
      display_write("got bluetooth update", DISPLAY_LINE_0);
      if (updated) {
        printf("updating variables...\n");
        y_pos_readable = ypos;
        x_pos_readable = xpos;
        angle_readable = angle;
        //index_readable = index;
        // updated = false;
      } else {
        printf("bad bluetooth update\n");
      }
    }
}

extern void ble_evt_connected(ble_evt_t const* p_ble_evt) {
    display_write("BLE CONNECTED", DISPLAY_LINE_0);
    ble_connected = true;
    nrf_delay_ms(1000);
}

extern void ble_evt_disconnected(ble_evt_t const* p_ble_evt) {
    display_write("BLE DISCONNECTED :(", DISPLAY_LINE_0);
    ble_connected = false;
    nrf_delay_ms(1000);
}

void print_state(states current_state){
  switch(current_state){
  case OFF:
    display_write("OFF", DISPLAY_LINE_0);
    break;
    }
}


int get_pos_angle(int *x, int *y, int *angle, int timeout) {
    int timer = 0;
    while (!updated) { // dumb spin lock... lmk if you have better idea...
        nrf_delay_ms(1000);
        timer += 1;
        display_write("waiting 4 update...", DISPLAY_LINE_0);
        if (timer >= timeout) {
          return -1;
        }

    }
    *x = x_pos_readable;
    *y = y_pos_readable;
    *angle = angle_readable;
    updated = false;
    return 0;
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);
 
  simple_ble_add_service(&angle_service);

  // TODO: Register your characteristics

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
  printf("Log initialized14!\n"); 
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
    //int status = kobukiSensorPoll(&sensors);

    // TODO: complete state machine
    switch(state) {
      case OFF: {
        print_state(state);

        // transition logic
        if (is_button_pressed(&sensors)) {
          //state = NEWSTATE;
        } else {
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
        }

        // try to read bluetooth measurements
        int my_x =0 , my_y = 0, my_angle = 0;
        int succ = get_pos_angle(&my_x, &my_y, &my_angle, 10);
        if (succ == 0) {
          display_write("read ble measurement", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf, 16, "%x", my_x);
          //printf("%d\n", my_x);
          display_write(buf, DISPLAY_LINE_1);
          nrf_delay_ms(1000);
        } else {
          display_write("cannot read", DISPLAY_LINE_1);
          nrf_delay_ms(1000);
        }


        break; // each case needs to end with break!
      }
    }
  }
}
