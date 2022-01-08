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
#include "can.h"
#include <math.h>

// Define pins for SPI (to CAN)
#define SPI_PORT spi0
#define SPI_MISO 16
#define SPI_CS   17
#define SPI_CLK  18
#define SPI_MOSI 19
#define CAN_INT  20
#define CAN_CLK  21 // 8MHz clock for CAN

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

void CAN_transmit(uint16_t id, uint8_t* data, uint8_t length) {
  CAN_reg_write(REG_TXBnSIDH(0), id >> 3); // Set CAN ID
  CAN_reg_write(REG_TXBnSIDL(0), id << 5); // Set CAN ID
  CAN_reg_write(REG_TXBnEID8(0), 0x00);    // Extended ID
  CAN_reg_write(REG_TXBnEID0(0), 0x00);    // Extended ID

  CAN_reg_write(REG_TXBnDLC(0), length);   // Frame length

  for (int i = 0; i < length; i++) {       // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);    // Start sending
  busy_wait_us(1000); // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);    // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00); // Clear interrupt flag
}

void gpio_callback(uint gpio, uint32_t events) {
  CAN_receive(can_data_buffer);
  gpio_acknowledge_irq(CAN_INT, GPIO_IRQ_LEVEL_LOW);
}

int main()
{
  // Set system clock to 80MHz, this seems like a reasonable value for the 4MHz data
  set_sys_clock_khz(80000, true);
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
  clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS_BITS;
  clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS;

  // Configure CAN interrupt input pin
  gpio_init(CAN_INT);
  gpio_set_dir(CAN_INT,GPIO_IN);
  gpio_disable_pulls(CAN_INT);
  gpio_set_irq_enabled_with_callback(CAN_INT, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);

  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

  // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure(0x4F0);

  // Main loop.
  while (1) {
  }
}
