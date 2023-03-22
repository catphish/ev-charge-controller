#include <math.h>
#include <stdio.h>

#include "can.h"
#include "charge_controller.pio.h"
#include "device/usbd.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/usb.h"
#include "hardware/rosc.h"
#include "hardware/spi.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "hardware/xosc.h"
#include "pico/sleep.h"
#include "pico/stdlib.h"

// Set the final charging voltage in units of 100mV
// For example 41 (4.1V) * 16 cells * 10 modules
uint16_t target_voltage = 41 * 16 * 5;

// Define pins for SPI (to CAN)
#define SPI_PORT spi0
#define SPI_MISO 16
#define SPI_CS 17
#define SPI_CLK 18
#define SPI_MOSI 19
#define CAN_INT 20
#define CAN_CLK 21  // 8MHz clock for CAN
#define CAN_SLEEP 22

// EVSE pins
#define EVSE_PP 26
#define EVSE_CP 27
#define EVSE_OUT 28

// GPIO pins
#define IN1 14
#define IN2 13
#define IN3 12
#define IN4 10
#define OUT1 9
#define OUT2 6
#define OUT3 4
#define OUT4 2

#define IGNITION_IN IN3  // Detect ignition live
#define HEAT_IN IN4      // Input from heat switch
#define START_IN IN1     // Inut from start button

#define DC_DC_OUT OUT4  // Output to DC-DC converter enable relay
#define HEAT_OUT OUT2   // Output to heater relay

// State
int32_t pwm_value;
uint32_t pack_voltage;
uint16_t max_temp;
uint16_t min_temp;
uint16_t max_cell;
uint16_t min_cell;
uint16_t data_timer;
uint8_t error;
uint8_t charging;

void SPI_configure() {
  spi_init(SPI_PORT, 1000000);
  spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
  gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
  gpio_init(SPI_CS);
  gpio_set_dir(SPI_CS, GPIO_OUT);
  gpio_put(SPI_CS, 1);
}

void CAN_reset() {
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
  gpio_put(SPI_CS, 1);
  busy_wait_us(100);
}

uint8_t CAN_reg_read(uint8_t reg) {
  uint8_t data;
  gpio_put(SPI_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
  spi_read_blocking(SPI_PORT, 0, &data, 1);
  gpio_put(SPI_CS, 1);
  return (data);
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

void CAN_configure() {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from
  // https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_reg_write(REG_CNF1, 0x00);
  CAN_reg_write(REG_CNF2, 0x90);
  CAN_reg_write(REG_CNF3, 0x02);

  // Enable Filters
  CAN_reg_write(REG_RXBnCTRL(0), 0);  // Enable filters, no rollover
  CAN_reg_write(REG_RXBnCTRL(1), 0);
  // Set masks RXM0 and RXM1 to exact match (0x7FF)
  for (int n = 0; n < 2; n++) {
    uint32_t mask = 0x7FF;
    CAN_reg_write(REG_RXMnSIDH(n), mask >> 3);
    CAN_reg_write(REG_RXMnSIDL(n), mask << 5);
    CAN_reg_write(REG_RXMnEID8(n), 0);
    CAN_reg_write(REG_RXMnEID0(n), 0);
  }
  // Set up filters RXF0 and RFX2 to match 2 addresses
  uint32_t addr = 0x4F0;
  CAN_reg_write(REG_RXFnSIDH(0), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(0), addr << 5);
  CAN_reg_write(REG_RXFnEID8(0), 0);
  CAN_reg_write(REG_RXFnEID0(0), 0);

  addr = 0x4F1;
  CAN_reg_write(REG_RXFnSIDH(2), addr >> 3);
  CAN_reg_write(REG_RXFnSIDL(2), addr << 5);
  CAN_reg_write(REG_RXFnEID8(2), 0);
  CAN_reg_write(REG_RXFnEID0(2), 0);

  // Enable receive interrupts
  // CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
}

uint8_t CAN_receive() {
  uint8_t intf = CAN_reg_read(REG_CANINTF);
  uint8_t rtr;
  uint8_t n;  // One of two receive buffers
  if (intf & FLAG_RXnIF(0)) {
    n = 0;
  } else if (intf & FLAG_RXnIF(1)) {
    n = 1;
  } else {
    return 0;
  }
  rtr = (CAN_reg_read(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
  uint8_t dlc = CAN_reg_read(REG_RXBnDLC(n)) & 0x0f;

  uint8_t length;
  if (!rtr) {
    data_timer = 0;
    uint8_t received_data[8];
    for (int i = 0; i < 8; i++) received_data[i] = CAN_reg_read(REG_RXBnD0(n) + i);
    if (!n) {
      // 0x4F0
      printf("Pack voltage received\n");
      pack_voltage = (received_data[0] << 24) | (received_data[1] << 16) | (received_data[2] << 8) | (received_data[3] << 0);
      pack_voltage /= 3;
    } else {
      // 0x4F1
      printf("Cell data received\n");
      max_cell = (received_data[0] << 8) | (received_data[1] << 0);
      min_cell = (received_data[2] << 8) | (received_data[3] << 0);
      max_temp = (received_data[4] << 8) | (received_data[5] << 0);
      min_temp = (received_data[6] << 8) | (received_data[7] << 0);
    }
  }

  CAN_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
  return (length);
}

void CAN_transmit(uint8_t ext, uint32_t id, uint8_t* data, uint8_t length) {
  if (ext) {
    CAN_reg_write(REG_TXBnEID0(0), id);
    CAN_reg_write(REG_TXBnEID8(0), id >> 8);
    CAN_reg_write(REG_TXBnSIDL(0), ((id >> 16) & 3) | FLAG_EXIDE | (((id >> 18) & 7) << 5));
    CAN_reg_write(REG_TXBnSIDH(0), id >> 21);
  } else {
    CAN_reg_write(REG_TXBnSIDH(0), id >> 3);  // Set CAN ID
    CAN_reg_write(REG_TXBnSIDL(0), id << 5);  // Set CAN ID
    CAN_reg_write(REG_TXBnEID8(0), 0x00);     // Extended ID
    CAN_reg_write(REG_TXBnEID0(0), 0x00);     // Extended ID
  }

  CAN_reg_write(REG_TXBnDLC(0), length);  // Frame length

  for (int i = 0; i < length; i++) {  // Write the frame data
    CAN_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_reg_write(REG_TXBnCTRL(0), 0x08);              // Start sending
  busy_wait_us(1000);                                // Allow up to 1ms to transmit
  CAN_reg_write(REG_TXBnCTRL(0), 0);                 // Stop sending
  CAN_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00);  // Clear interrupt flag
}

// Interrupt for CAN and dummy interrupt for hardware wakeup
void gpio_callback() {
  //  CAN_receive();
  gpio_acknowledge_irq(CAN_INT, GPIO_IRQ_LEVEL_LOW);
}

void reconfigure_clocks() {
  // Clock the peripherals, ref clk, and rtc from the 12MHz crystal oscillator
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 12000000);
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, 12000000, 12000000);
  clock_configure(clk_rtc, 0, CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 12000000, 46875);
  // Shut down unused clocks, PLLs and oscillators
  clock_stop(clk_adc);
  rosc_disable();
  // Disable more clocks when sleeping
  clocks_hw->sleep_en0 = CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB_BITS;
  clocks_hw->sleep_en1 = CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS | CLOCKS_SLEEP_EN1_CLK_USB_USBCTRL_BITS | CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS;
}

void deep_sleep() {
  // Make 100% sure everything is shut down
  gpio_put(HEAT_OUT, 0);
  gpio_put(DC_DC_OUT, 0);
  gpio_put(EVSE_OUT, 0);

  // Wait 100ms for CAN to finish transmitting
  busy_wait_ms(100);
  // Deep sleep until woken by hardware
  tud_disconnect();  // Disconnect USB
  CAN_reg_write(REG_CANCTRL, MODE_SLEEP);
  gpio_put(CAN_SLEEP, 1);  // Sleep the CAN transceiver
  uint32_t s = save_and_disable_interrupts();
  gpio_set_irq_enabled_with_callback(EVSE_CP, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);
  gpio_set_irq_enabled_with_callback(IGNITION_IN, GPIO_IRQ_LEVEL_HIGH, true, &gpio_callback);
  gpio_set_dormant_irq_enabled(EVSE_CP, GPIO_IRQ_LEVEL_LOW, true);
  gpio_set_dormant_irq_enabled(IGNITION_IN, GPIO_IRQ_LEVEL_HIGH, true);
  clocks_hw->sleep_en0 = 0;
  clocks_hw->sleep_en1 = 0;
  xosc_dormant();
  reconfigure_clocks();

  // Invalidate data
  pack_voltage = 0;
  max_cell = 0;
  min_cell = 0;
  max_temp = 0;
  min_temp = 0;

  gpio_set_irq_enabled_with_callback(EVSE_CP, GPIO_IRQ_LEVEL_LOW, false, &gpio_callback);
  gpio_set_irq_enabled_with_callback(IGNITION_IN, GPIO_IRQ_LEVEL_HIGH, false, &gpio_callback);
  restore_interrupts(s);
  SPI_configure();
  gpio_put(CAN_SLEEP, 0);  // Wake the CAN transceiver
  CAN_reg_write(REG_CANCTRL, MODE_NORMAL);
  tud_connect();
  stdio_usb_init();  // Restore USB
}

int usb_suspended() { return (usb_hw->sie_status & USB_SIE_STATUS_SUSPENDED_BITS); }

float temperature(uint16_t adc) {
  float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
  float t = log(r) * -30.5280964239816f + 95.6841501312447f;
  return t;
}

int main() {
  // Set system clock to 80MHz
  set_sys_clock_khz(80000, true);
  stdio_init_all();
  reconfigure_clocks();

  busy_wait_ms(1000);

  // CP output
  gpio_init(EVSE_OUT);
  gpio_set_dir(EVSE_OUT, GPIO_OUT);
  gpio_put(EVSE_OUT, 0);

  // Ignition input
  gpio_init(IGNITION_IN);
  gpio_set_dir(IGNITION_IN, GPIO_IN);
  gpio_disable_pulls(IGNITION_IN);

  // DC-DC output
  gpio_init(DC_DC_OUT);
  gpio_set_dir(DC_DC_OUT, GPIO_OUT);
  gpio_put(DC_DC_OUT, 0);

  // Heater output
  gpio_init(HEAT_OUT);
  gpio_set_dir(HEAT_OUT, GPIO_OUT);
  gpio_put(HEAT_OUT, 0);

  // Configure CAN transceiver sleep line
  gpio_init(CAN_SLEEP);
  gpio_set_dir(CAN_SLEEP, GPIO_OUT);
  gpio_put(CAN_SLEEP, 0);  // Logic low to wake transceiver

  // Output 8MHz square wave on CAN_CLK pin
  clock_gpio_init(CAN_CLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

  // Configure SPI to communicate with CAN
  SPI_configure();
  // Set up CAN to receive messages
  CAN_reset();
  CAN_configure();
  // Configure CAN interrupt input pin
  // gpio_init(CAN_INT);
  // gpio_set_dir(CAN_INT, GPIO_IN);
  // gpio_disable_pulls(CAN_INT);
  // gpio_set_irq_enabled_with_callback(CAN_INT, GPIO_IRQ_LEVEL_LOW, true,
  //                                    &gpio_callback);

  uint32_t dma_channel_1 = dma_claim_unused_channel(true);
  uint32_t dma_channel_2 = dma_claim_unused_channel(true);
  dma_channel_config dma_config_1 = dma_channel_get_default_config(dma_channel_1);
  dma_channel_config dma_config_2 = dma_channel_get_default_config(dma_channel_2);
  channel_config_set_read_increment(&dma_config_1, false);
  channel_config_set_read_increment(&dma_config_2, false);
  channel_config_set_write_increment(&dma_config_1, false);
  channel_config_set_write_increment(&dma_config_2, false);
  channel_config_set_transfer_data_size(&dma_config_1, DMA_SIZE_32);
  channel_config_set_transfer_data_size(&dma_config_2, DMA_SIZE_32);
  channel_config_set_chain_to(&dma_config_1, dma_channel_2);
  channel_config_set_chain_to(&dma_config_2, dma_channel_1);
  channel_config_set_dreq(&dma_config_1, pio_get_dreq(pio0, 0, false));
  channel_config_set_dreq(&dma_config_2, pio_get_dreq(pio0, 0, false));
  dma_channel_configure(dma_channel_1, &dma_config_1, &pwm_value, &pio0->rxf[0], 1, false);
  dma_channel_configure(dma_channel_2, &dma_config_2, &pwm_value, &pio0->rxf[0], 1, true);

  uint32_t offset = pio_add_program(pio0, &pwm_measurement_program);
  pwm_measurement_program_init(pio0, 0, offset, EVSE_CP);

  // Timeout
  pio_sm_put_blocking(pio0, 0, 100000);

  uint8_t heat_allowed = 0;

  // Main loop.
  while (1) {
    // Sleep for a minimum of 100ms per loop
    busy_wait_ms(50);
    CAN_receive();
    busy_wait_ms(50);
    CAN_receive();
    int32_t pwm_value_cache;
    pwm_value_cache = pwm_value;
    uint32_t ac_current = 100000 - pwm_value_cache - 138;
    ac_current = ac_current * 2250 / 1000;
    // Magic number below is 3145728 * efficiency (0.833) = 2621440
    uint32_t dc_current = ac_current * (2621440 >> 8) / (pack_voltage >> 8);
    uint8_t ignition = gpio_get(IGNITION_IN);
    uint8_t heat = gpio_get(HEAT_IN);
    uint8_t button = gpio_get(START_IN);
    printf("IGN: %i HEAT: %i button: %i\n", ignition, heat, button);

    // Disable heater below 3.3V, re-enable at 3.4V
    if (min_cell > 44564) heat_allowed = 1;
    if (min_cell < 43253) heat_allowed = 0;
    // Enable heater if ignition and heat inputs are set
    gpio_put(HEAT_OUT, heat_allowed && ignition && heat);

    // Invalidate data if timer expires
    data_timer++;
    if (data_timer > 100) {
      pack_voltage = 0;
      max_cell = 0;
      min_cell = 0;
      max_temp = 0;
      min_temp = 0;
    }
    printf("PWM value: %i\n", pwm_value_cache);
    if (pwm_value_cache > 0 && pwm_value_cache < 80000) {
      printf("Ambiguous PWM reading\n");
    } else if (pwm_value_cache > 99860) {
      printf("Ambiguous PWM reading\n");
    } else if (pwm_value_cache > 0) {
      CAN_transmit(0, 0x4E0, (uint8_t[]){0},
                   1);  // No inputs set on drive unit
      if ((!pack_voltage || !max_cell || !max_temp || !min_temp) && charging) {
        error = 1;                    // Failed to receive CAN data while charging
      } else if (max_cell > 53083) {  // Stop charging when one
        error = 2;                    // cell reaches 4.05V
      } else if (max_temp && temperature(max_temp) > 45.f) {
        error = 3;  // Over temperature
      } else if (min_temp && temperature(min_temp) < 2.f) {
        error = 4;  // Under temperature
      }
      if (error) {
        if (error == 1) printf("BMS: CAN data timeout\n");
        if (error == 2) printf("BMS: Cell above 4.0V - charging not permitted\n");
        if (error == 3) printf("BMS: Temperature above 45C - charging not permitted\n");
        if (error == 4) printf("BMS: Temperature below 2C - charging not permitted\n");
        gpio_put(EVSE_OUT, 0);
        gpio_put(DC_DC_OUT, ignition);
        // Bit 5 is 1 (stop)
        CAN_transmit(1, 0x1806E5F4, (uint8_t[]){target_voltage >> 8, target_voltage, 0, 0, 1, 0, 0, 0}, 8);
      } else if (!pack_voltage || !max_cell || !max_temp || !min_temp) {
        printf("BMS: Waiting for CAN data\n");
        gpio_put(EVSE_OUT, 0);
        gpio_put(DC_DC_OUT, 1);
        // Bit 5 is 1 (stop)
        CAN_transmit(1, 0x1806E5F4, (uint8_t[]){target_voltage >> 8, target_voltage, 0, 0, 1, 0, 0, 0}, 8);
      } else {
        printf("Charging\n");
        printf("  DC Voltage: %i.%iV\n", pack_voltage / 13107, pack_voltage % 13107);
        printf("  AC Current Limit: %i.%iA\n", ac_current / 1000, ac_current % 1000);
        printf("  DC Current Limit: %i.%iA\n", dc_current / 1000, dc_current % 1000);
        printf("  Temperature: %.2fC - %.2fC\n", temperature(min_temp), temperature(max_temp));
        gpio_put(EVSE_OUT, 1);
        gpio_put(DC_DC_OUT, 1);
        // Bit 5 is 0 (charging)
        CAN_transmit(1, 0x1806E5F4, (uint8_t[]){target_voltage >> 8, target_voltage, (dc_current / 100) >> 8, dc_current / 100, 0, 0, 0, 0}, 8);
        charging = 1;
      }
    } else {
      // EVSE unplugged, clear errors and stop charging
      error = 0;
      charging = 0;
      // Bit 5 is 1 (stop)
      CAN_transmit(1, 0x1806E5F4, (uint8_t[]){target_voltage >> 8, target_voltage, 0, 0, 1, 0, 0, 0}, 8);
      printf("EVSE: NO SIGNAL\n");
      gpio_put(EVSE_OUT, 0);
      gpio_put(DC_DC_OUT, ignition);

      // Map start button to drive unit via CAN
      if (button)
        CAN_transmit(0, 0x4E0, (uint8_t[]){2},
                     1);  // Start bit set on drive unit
      else
        CAN_transmit(0, 0x4E0, (uint8_t[]){0},
                     1);  // No inputs set on drive unit

      // Go into low power sleep unless USB mode or ignition
      if (!ignition && usb_suspended()) deep_sleep();
    }
  }
}
