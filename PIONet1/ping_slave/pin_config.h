/**
 * Pin Definitions for PIO SPI Ping Test
 * 
 * IMPORTANT: Both boards use the same pin assignments!
 * 
 * Wiring between Board A and Board B:
 * 
 *   Board A                      Board B
 *   ───────                      ───────
 *   GPIO 2  (TX_CLK)  ─────────> GPIO 11 (RX_CLK)
 *   GPIO 3  (TX_CS)   ─────────> GPIO 10 (RX_CS)
 *   GPIO 4  (TX_DATA) ─────────> GPIO 12 (RX_DATA)
 *   
 *   GPIO 11 (RX_CLK)  <───────── GPIO 2  (TX_CLK)
 *   GPIO 10 (RX_CS)   <───────── GPIO 3  (TX_CS)
 *   GPIO 12 (RX_DATA) <───────── GPIO 4  (TX_DATA)
 *   
 *   GND ──────────────────────── GND
 * 
 * Total: 6 signal wires + 1 ground = 7 wires
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// TX pins (directly active low CS)
// CLK and CS must be adjacent
#define TX_CLK_PIN    2       // Base for side-set
#define TX_CS_PIN     3       // Base + 1 (automatic)
#define TX_DATA_PIN   4       // Can be anywhere

// RX pins (CS, CLK, DATA must be consecutive)
#define RX_CS_PIN     10      // Base
#define RX_CLK_PIN    11      // Base + 1 (automatic)
#define RX_DATA_PIN   12      // Base + 2 (automatic)

// Communication settings
#define SPI_FREQ_HZ   10000000  // 10 MHz

#endif // PIN_CONFIG_H
