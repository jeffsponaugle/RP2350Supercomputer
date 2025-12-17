/**
 * PIO SPI Ping Master
 * 
 * Sends ping bytes to slave, waits for echo response.
 * Measures round-trip time and reports statistics.
 * 
 * Flash this onto Board A (the "master").
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pio_spi_dma.h"
#include "pin_config.h"

// Test settings
#define PING_INTERVAL_MS    100     // Time between pings
#define PING_TIMEOUT_MS     50      // How long to wait for response
#define STATS_INTERVAL      10      // Print stats every N pings

// Instances
static pio_spi_dma_tx_inst_t tx;
static pio_spi_dma_rx_inst_t rx;

// Statistics
static uint32_t pings_sent = 0;
static uint32_t pongs_received = 0;
static uint32_t timeouts = 0;
static uint32_t errors = 0;
static uint64_t total_rtt_us = 0;
static uint32_t min_rtt_us = UINT32_MAX;
static uint32_t max_rtt_us = 0;

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

static void print_stats(void) {
    printf("\n--- Ping Statistics ---\n");
    printf("Sent: %lu, Received: %lu, Timeouts: %lu, Errors: %lu\n",
           pings_sent, pongs_received, timeouts, errors);
    
    if (pongs_received > 0) {
        uint32_t avg_rtt = (uint32_t)(total_rtt_us / pongs_received);
        printf("RTT min/avg/max = %lu/%lu/%lu us\n", min_rtt_us, avg_rtt, max_rtt_us);
        
        float loss = 100.0f * (float)(pings_sent - pongs_received) / (float)pings_sent;
        printf("Packet loss: %.1f%%\n", loss);
    }
    printf("\n");
}

static void print_wiring_diagram(void) {
    printf("\n");
    printf("============================================\n");
    printf("       PIO SPI PING MASTER (Board A)\n");
    printf("============================================\n");
    printf("\n");
    printf("Wiring to Slave (Board B):\n");
    printf("\n");
    printf("  Board A              Board B\n");
    printf("  ────────             ────────\n");
    printf("  GPIO %2d (TX_CLK)  ──> GPIO %2d (RX_CLK)\n", TX_CLK_PIN, RX_CLK_PIN);
    printf("  GPIO %2d (TX_CS)   ──> GPIO %2d (RX_CS)\n", TX_CS_PIN, RX_CS_PIN);
    printf("  GPIO %2d (TX_DATA) ──> GPIO %2d (RX_DATA)\n", TX_DATA_PIN, RX_DATA_PIN);
    printf("\n");
    printf("  GPIO %2d (RX_CLK)  <── GPIO %2d (TX_CLK)\n", RX_CLK_PIN, TX_CLK_PIN);
    printf("  GPIO %2d (RX_CS)   <── GPIO %2d (TX_CS)\n", RX_CS_PIN, TX_CS_PIN);
    printf("  GPIO %2d (RX_DATA) <── GPIO %2d (TX_DATA)\n", RX_DATA_PIN, TX_DATA_PIN);
    printf("\n");
    printf("  GND ──────────────────── GND\n");
    printf("\n");
    printf("Speed: %.1f MHz\n", SPI_FREQ_HZ / 1000000.0f);
    printf("\n");
}

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Wait for USB connection and give time to open terminal
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
    
    // Initialize TX
    printf("Initializing TX... ");
    tx = pio_spi_dma_tx_init(pio0, 0, TX_CLK_PIN, TX_DATA_PIN, SPI_FREQ_HZ);
    if (tx.dma_chan < 0) {
        printf("FAILED!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("OK (DMA ch %d)\n", tx.dma_chan);
    
    // Initialize RX
    printf("Initializing RX... ");
    rx = pio_spi_dma_rx_init(pio0, 1, RX_CS_PIN);
    if (rx.dma_chan < 0) {
        printf("FAILED!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("OK (DMA ch %d)\n", rx.dma_chan);
    
    printf("\nStarting ping test...\n");
    printf("Press any key to show statistics\n\n");
    
    // Ping loop
    uint8_t tx_byte;
    uint8_t rx_byte;
    uint8_t sequence = 0;
    
    while (1) {
        // Prepare ping byte (use sequence number as payload)
        tx_byte = sequence++;
        
        // Flush any stale RX data
        pio_spi_dma_rx_flush(&rx);
        
        // Start RX (waiting for response)
        pio_spi_dma_rx_start(&rx, &rx_byte, 1);
        
        // Record start time
        absolute_time_t start = get_absolute_time();
        
        // Send ping
        pio_spi_dma_tx_blocking(&tx, &tx_byte, 1);
        pings_sent++;
        
        // Wait for response with timeout
        absolute_time_t deadline = make_timeout_time_ms(PING_TIMEOUT_MS);
        bool got_response = false;
        
        while (!time_reached(deadline)) {
            if (!pio_spi_dma_rx_busy(&rx)) {
                got_response = true;
                break;
            }
            tight_loop_contents();
        }
        
        if (got_response) {
            // Calculate RTT
            absolute_time_t end = get_absolute_time();
            uint32_t rtt_us = (uint32_t)absolute_time_diff_us(start, end);
            
            // Verify response
            if (rx_byte == tx_byte) {
                pongs_received++;
                total_rtt_us += rtt_us;
                if (rtt_us < min_rtt_us) min_rtt_us = rtt_us;
                if (rtt_us > max_rtt_us) max_rtt_us = rtt_us;
                
                printf("PING seq=%3d: reply in %lu us\n", tx_byte, rtt_us);
                led_toggle();
            } else {
                errors++;
                printf("PING seq=%3d: ERROR sent=0x%02X got=0x%02X\n", 
                       tx_byte, tx_byte, rx_byte);
            }
        } else {
            // Timeout
            timeouts++;
            pio_spi_dma_rx_abort(&rx);
            printf("PING seq=%3d: TIMEOUT\n", tx_byte);
        }
        
        // Print stats periodically
        if (pings_sent % STATS_INTERVAL == 0) {
            print_stats();
        }
        
        // Check for keypress to show stats
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            print_stats();
        }
        
        // Wait before next ping
        sleep_ms(PING_INTERVAL_MS);
    }
    
    return 0;
}
