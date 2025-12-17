/**
 * PIO-based SPI-like TX/RX with DMA for RP2350
 * 
 * Uses DMA to transfer data to/from PIO FIFOs automatically.
 * CPU just sets up buffers, DMA handles the rest.
 *
 * Features:
 *   - TX: DMA feeds PIO FIFO from memory buffer
 *   - RX: DMA drains PIO FIFO to memory buffer
 *   - Interrupt on transfer complete
 *   - No flow control needed - DMA keeps up with PIO
 *
 * Signals (3 wires per direction):
 *   CS   (TX→RX) - Chip select, active LOW, frames each byte
 *   CLK  (TX→RX) - Clock, data sampled on rising edge
 *   DATA (TX→RX) - Data, MSB first
 *
 * Pin Requirements:
 *   TX: CLK at base, CS at base+1 (consecutive), DATA anywhere
 *   RX: CS at base, CLK at base+1, DATA at base+2 (all consecutive)
 *
 * Timing: 12 cycles/bit, ~12 MHz max, recommend 10 MHz
 */

#ifndef PIO_SPI_CS_DMA_H
#define PIO_SPI_CS_DMA_H

#include "hardware/pio.h"
#include "hardware/dma.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Callback Type
// ============================================================================

/** Callback function type for transfer complete notifications */
typedef void (*pio_spi_dma_callback_t)(void *user_data);

// ============================================================================
// Instance Structures
// ============================================================================

typedef struct {
    PIO pio;
    uint sm;
    uint pio_offset;
    uint dma_chan;
    volatile bool busy;
    pio_spi_dma_callback_t callback;
    void *callback_data;
} pio_spi_dma_tx_inst_t;

typedef struct {
    PIO pio;
    uint sm;
    uint pio_offset;
    uint dma_chan;
    volatile bool busy;
    pio_spi_dma_callback_t callback;
    void *callback_data;
} pio_spi_dma_rx_inst_t;

// ============================================================================
// TX Initialization
// ============================================================================

/**
 * Initialize SPI TX with DMA
 * 
 * @param pio       PIO instance (pio0, pio1, or pio2)
 * @param sm        State machine index (0-3)
 * @param pin_clk   GPIO for CLK output (CS is pin_clk + 1)
 * @param pin_data  GPIO for DATA output (any pin)
 * @param freq_hz   Bit rate in Hz (max ~12 MHz)
 * @return          Initialized instance (dma_chan = -1 on failure)
 */
pio_spi_dma_tx_inst_t pio_spi_dma_tx_init(PIO pio, uint sm,
                                           uint pin_clk, uint pin_data,
                                           float freq_hz);

// ============================================================================
// RX Initialization
// ============================================================================

/**
 * Initialize SPI RX with DMA
 * 
 * @param pio       PIO instance (pio0, pio1, or pio2)
 * @param sm        State machine index (0-3)
 * @param pin_cs    GPIO for CS input (CLK=pin_cs+1, DATA=pin_cs+2)
 * @return          Initialized instance (dma_chan = -1 on failure)
 */
pio_spi_dma_rx_inst_t pio_spi_dma_rx_init(PIO pio, uint sm, uint pin_cs);

// ============================================================================
// TX Functions
// ============================================================================

/**
 * Start DMA transfer from buffer to TX
 * 
 * @param inst      TX instance
 * @param data      Source buffer (must remain valid until transfer completes)
 * @param len       Number of bytes to send
 * 
 * Returns immediately. Use pio_spi_dma_tx_busy() or callback to detect completion.
 */
void pio_spi_dma_tx_start(pio_spi_dma_tx_inst_t *inst, const uint8_t *data, size_t len);

/**
 * Check if TX DMA transfer is in progress
 */
static inline bool pio_spi_dma_tx_busy(pio_spi_dma_tx_inst_t *inst) {
    return inst->busy || dma_channel_is_busy(inst->dma_chan);
}

/**
 * Wait for TX DMA transfer to complete
 */
void pio_spi_dma_tx_wait(pio_spi_dma_tx_inst_t *inst);

/**
 * Send buffer and wait for completion (blocking)
 */
void pio_spi_dma_tx_blocking(pio_spi_dma_tx_inst_t *inst, const uint8_t *data, size_t len);

/**
 * Set callback for TX transfer complete
 * 
 * @param inst      TX instance
 * @param callback  Function to call on completion (NULL to disable)
 * @param user_data Passed to callback
 */
void pio_spi_dma_tx_set_callback(pio_spi_dma_tx_inst_t *inst,
                                  pio_spi_dma_callback_t callback,
                                  void *user_data);

/**
 * Abort any in-progress TX transfer
 */
void pio_spi_dma_tx_abort(pio_spi_dma_tx_inst_t *inst);

/**
 * Disable TX and release resources
 */
void pio_spi_dma_tx_deinit(pio_spi_dma_tx_inst_t *inst);

// ============================================================================
// RX Functions
// ============================================================================

/**
 * Start DMA transfer from RX to buffer
 * 
 * @param inst      RX instance
 * @param data      Destination buffer (must remain valid until transfer completes)
 * @param len       Number of bytes to receive
 * 
 * Returns immediately. Use pio_spi_dma_rx_busy() or callback to detect completion.
 */
void pio_spi_dma_rx_start(pio_spi_dma_rx_inst_t *inst, uint8_t *data, size_t len);

/**
 * Check if RX DMA transfer is in progress
 */
static inline bool pio_spi_dma_rx_busy(pio_spi_dma_rx_inst_t *inst) {
    return inst->busy || dma_channel_is_busy(inst->dma_chan);
}

/**
 * Wait for RX DMA transfer to complete
 */
void pio_spi_dma_rx_wait(pio_spi_dma_rx_inst_t *inst);

/**
 * Receive into buffer and wait for completion (blocking)
 */
void pio_spi_dma_rx_blocking(pio_spi_dma_rx_inst_t *inst, uint8_t *data, size_t len);

/**
 * Set callback for RX transfer complete
 * 
 * @param inst      RX instance
 * @param callback  Function to call on completion (NULL to disable)
 * @param user_data Passed to callback
 */
void pio_spi_dma_rx_set_callback(pio_spi_dma_rx_inst_t *inst,
                                  pio_spi_dma_callback_t callback,
                                  void *user_data);

/**
 * Get number of bytes remaining in current RX transfer
 */
static inline size_t pio_spi_dma_rx_remaining(pio_spi_dma_rx_inst_t *inst) {
    return dma_channel_hw_addr(inst->dma_chan)->transfer_count;
}

/**
 * Abort any in-progress RX transfer
 */
void pio_spi_dma_rx_abort(pio_spi_dma_rx_inst_t *inst);

/**
 * Flush RX FIFO (discards any pending data)
 */
void pio_spi_dma_rx_flush(pio_spi_dma_rx_inst_t *inst);

/**
 * Disable RX and release resources
 */
void pio_spi_dma_rx_deinit(pio_spi_dma_rx_inst_t *inst);

#ifdef __cplusplus
}
#endif

#endif // PIO_SPI_CS_DMA_H
