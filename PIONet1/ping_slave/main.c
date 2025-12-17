/**
 * PIO SPI Ping Slave (Echo)
 * 
 * Listens for incoming bytes and echoes them back.
 * 
 * Flash this onto Board B (the "slave").
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pio_spi_dma.h"
#include "pin_config.h"

// Instances
static pio_spi_dma_tx_inst_t tx;
static pio_spi_dma_rx_inst_t rx;

// Statistics
static uint32_t bytes_received = 0;
static uint32_t bytes_echoed = 0;

// LED for visual feedback
#define LED_PIN PICO_DEFAULT_LED_PIN

static void led_init(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

static void led_toggle(void) {
    gpio_xor_mask(1u << LED_PIN);
}

static void print_wiring_diagram(void) {
    printf("\n");
    printf("============================================\n");
    printf("       PIO SPI PING SLAVE (Board B)\n");
    printf("============================================\n");
    printf("\n");
    printf("Wiring to Master (Board A):\n");
    printf("\n");
    printf("  Board B              Board A\n");
    printf("  ────────             ────────\n");
    printf("  GPIO %2d (RX_CS)   <── GPIO %2d (TX_CS)\n", RX_CS_PIN, TX_CS_PIN);
    printf("  GPIO %2d (RX_CLK)  <── GPIO %2d (TX_CLK)\n", RX_CLK_PIN, TX_CLK_PIN);
    printf("  GPIO %2d (RX_DATA) <── GPIO %2d (TX_DATA)\n", RX_DATA_PIN, TX_DATA_PIN);
    printf("\n");
    printf("  GPIO %2d (TX_CLK)  ──> GPIO %2d (RX_CLK)\n", TX_CLK_PIN, RX_CLK_PIN);
    printf("  GPIO %2d (TX_CS)   ──> GPIO %2d (RX_CS)\n", TX_CS_PIN, RX_CS_PIN);
    printf("  GPIO %2d (TX_DATA) ──> GPIO %2d (RX_DATA)\n", TX_DATA_PIN, RX_DATA_PIN);
    printf("\n");
    printf("  GND ──────────────────── GND\n");
    printf("\n");
    printf("Speed: %.1f MHz\n", SPI_FREQ_HZ / 1000000.0f);
    printf("\n");
}

static void print_stats(void) {
    printf("\n--- Echo Statistics ---\n");
    printf("Received: %lu, Echoed: %lu\n\n", bytes_received, bytes_echoed);
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB connection
    sleep_ms(3000);
    
    print_wiring_diagram();
    
    // Print clock information
    uint32_t sys_clk = clock_get_hz(clk_sys);
    float tx_divider = (float)sys_clk / (12.0f * SPI_FREQ_HZ);
    if (tx_divider < 1.0f) tx_divider = 1.0f;
    float actual_bit_rate = (float)sys_clk / (12.0f * tx_divider);
    
    printf("Clock Configuration:\n");
    printf("  System clock:   %lu Hz (%.1f MHz)\n", sys_clk, sys_clk / 1000000.0f);
    printf("  TX clk divider: %.2f\n", tx_divider);
    printf("  RX clk divider: 1.00 (full speed)\n");
    printf("  Requested rate: %d Hz (%.1f MHz)\n", SPI_FREQ_HZ, SPI_FREQ_HZ / 1000000.0f);
    printf("  Actual TX rate: %.0f Hz (%.2f MHz)\n", actual_bit_rate, actual_bit_rate / 1000000.0f);
    printf("\n");
    
    // Initialize LED
    led_init();
    
    // Initialize TX (for sending echoes back)
    printf("Initializing TX... ");
    tx = pio_spi_dma_tx_init(pio0, 0, TX_CLK_PIN, TX_DATA_PIN, SPI_FREQ_HZ);
    if (tx.dma_chan < 0) {
        printf("FAILED!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("OK (DMA ch %d)\n", tx.dma_chan);
    
    // Initialize RX (for receiving pings)
    printf("Initializing RX... ");
    rx = pio_spi_dma_rx_init(pio0, 1, RX_CS_PIN);
    if (rx.dma_chan < 0) {
        printf("FAILED!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("OK (DMA ch %d)\n", rx.dma_chan);
    
    printf("\nWaiting for pings...\n");
    printf("Press any key to show statistics\n\n");
    
    // Buffer for received byte
    uint8_t rx_byte;
    
    // Main echo loop
    while (1) {
        // Start RX for one byte
        pio_spi_dma_rx_start(&rx, &rx_byte, 1);
        
        // Wait for data (with periodic checks for keypress)
        while (pio_spi_dma_rx_busy(&rx)) {
            // Check for keypress
            int c = getchar_timeout_us(1000);  // 1ms timeout
            if (c != PICO_ERROR_TIMEOUT) {
                print_stats();
            }
        }
        
        // Got a byte!
        bytes_received++;
        
        // Echo it back immediately
        pio_spi_dma_tx_blocking(&tx, &rx_byte, 1);
        bytes_echoed++;
        
        // Visual feedback
        led_toggle();
        
        // Print what we echoed
        printf("ECHO: seq=%3d (0x%02X)\n", rx_byte, rx_byte);
        
        // Print stats periodically
        if (bytes_received % 10 == 0) {
            print_stats();
        }
    }
    
    return 0;
}
