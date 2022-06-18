#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/sleep.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/rosc.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "bms.pio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/pwm.h"
#include "can.h"
#include <math.h>

// Define pins for SPI (to CAN)
#define SPI_PORT  spi0
#define SPI_MISO  16
#define SPI_CS    17
#define SPI_CLK   18
#define SPI_MOSI  19
#define CAN_CLK   21 // 8MHz clock for CAN

// General purpose outputs
#define OUT1 2
#define OUT2 4
#define OUT3 6
#define OUT4 9
#define IN1 14
#define IN2 13
#define IN3 12
#define IN4 10

// Buffers for received data
uint8_t can_data_buffer[16];

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0,0,SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET},1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return(data);
}

void CAN_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_CS, 1);
}

void CAN_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_CS, 1);
}

void CAN_configure(uint16_t id) {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 1<<2); // Enable rollover from BUF0 to BUF1
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks for RXB0 and RXB1 the same
  for(int n=0; n<2; n++) {
    uint16_t mask = 0x7ff;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set match ID for all filters the same
  for(int n=0; n<6; n++) {
    CAN_reg_write(REG_RXFnSIDH(n), id >> 3);
    CAN_reg_write(REG_RXFnSIDL(n), id << 5);
    CAN_reg_write(REG_RXFnEID8(n), 0);
    CAN_reg_write(REG_RXFnEID0(n), 0);
  }

  // Enable receive interrupts
  CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

uint8_t CAN_receive(uint8_t * can_rx_data) {
  uint8_t intf = CAN_reg_read(REG_CANINTF);
  uint8_t rtr;
  uint8_t n; // One of two receive buffers
  if(intf & FLAG_RXnIF(0)) {
    n = 0;
  } else if (intf & FLAG_RXnIF(1)) {
    n = 1;
  } else {
    return 0;
  }
  rtr = (CAN_reg_read(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
  uint8_t dlc = CAN_reg_read(REG_RXBnDLC(n)) & 0x0f;

  uint8_t length;
  if (rtr) {
    length = 0;
  } else {
    length = dlc;

    for (int i = 0; i < length; i++)
      can_rx_data[i] = CAN_reg_read(REG_RXBnD0(n) + i);
  }

  CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
  return(length);
}

float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}
void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
  // Shut down unused clocks, PLLs and oscillators
  clock_stop(clk_usb);
  clock_stop(clk_usb);
  clock_stop(clk_adc);
  pll_deinit(pll_usb);
  rosc_disable();
  // Disable more clocks when sleeping
  // clocks_hw->sleep_en0 = 0;
  // clocks_hw->sleep_en1 = CLOCKS_WAKE_EN1_CLK_SYS_TIMER_BITS;
}

int ignition_state = 0;

int main()
{
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz data
  set_sys_clock_khz(80000, true);
  reconfigure_clocks();
  //stdio_init_all();
  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

  // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure(0x4F1);

  // Configure PWM
  gpio_set_function(OUT2, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(OUT2);
  pwm_set_clkdiv(slice_num,1000);
  pwm_set_wrap(slice_num, 10486); // 4.0V - 3.2V
  pwm_set_enabled(slice_num, true);

  // OUT1 - Charger
  // OUT2 - Main Contactor
  // OUT3 - Fuel gauge PWM
  // OUT4 - Temperature warning
  gpio_init(OUT1);
  gpio_set_dir(OUT1, GPIO_OUT);
 // gpio_init(OUT2);
 // gpio_set_dir(OUT2, GPIO_OUT);
  gpio_init(OUT3);
  gpio_set_dir(OUT3, GPIO_OUT);
  gpio_init(OUT4);
  gpio_set_dir(OUT4, GPIO_OUT);

  gpio_init(IN3);
  gpio_set_dir(IN3, GPIO_IN);

  // Main loop.
  while (1) {
    //printf("%i\n", gpio_get(IN3));
    if(ignition_state) {
      // Ingition is on, wait for it to be turned off
      if(!gpio_get(IN3)) {
        gpio_put(OUT3, 0);
        ignition_state = 0;
      }
    } else {
      // Ignition is off, wait for it to be turned on
      if(gpio_get(IN3)) {
        sleep_ms(2000);
        gpio_put(OUT3, 1);
        ignition_state = 1;
      }
    }
    if(CAN_receive(can_data_buffer)) {
      uint16_t max_cell = ((uint16_t)can_data_buffer[0] << 8) | can_data_buffer[1];
      uint16_t min_cell = ((uint16_t)can_data_buffer[2] << 8) | can_data_buffer[3];
      uint16_t max_temp = ((uint16_t)can_data_buffer[4] << 8) | can_data_buffer[5];
      int32_t fuel_gauge = min_cell;
      fuel_gauge -= 41942;                        // Offset by 3.2V
      if(fuel_gauge > 10486) fuel_gauge = 10486;  // 4.0V is full
      if(fuel_gauge < 0) fuel_gauge = 0;          // 3.2V is empty
      pwm_set_gpio_level(OUT2, fuel_gauge);
      gpio_put(OUT4,temperature(max_temp) > 40.f); // Temperature warning at 40C
      // If voltage is below 3.2V, open the contactors.
      if(fuel_gauge == 0) {
        gpio_put(OUT3, 0);
      }
      // If max voltage is below 4.2V close charge contactor, else open it
      if(max_cell < 55049 && temperature(max_temp) > 5.f && temperature(max_temp < 50.f)) {
        gpio_put(OUT1, 1);
      } else {
        gpio_put(OUT1, 0);
        gpio_put(OUT4, 1);
        sleep_ms(20000);
      }
    }
  }
}
