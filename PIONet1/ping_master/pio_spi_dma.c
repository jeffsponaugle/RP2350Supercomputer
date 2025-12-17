/**
 * PIO-based SPI-like TX/RX with DMA for RP2350
 */

#include "pio_spi_dma.h"
#include "spi_tx_cs.pio.h"
#include "spi_rx_cs.pio.h"
#include "hardware/irq.h"

// ============================================================================
// IRQ Handling (internal)
// ============================================================================

// We need to track instances to dispatch IRQ callbacks
// Support up to 4 TX and 4 RX instances
static pio_spi_dma_tx_inst_t *tx_instances[4] = {NULL};
static pio_spi_dma_rx_inst_t *rx_instances[4] = {NULL};

static void dma_irq_handler(void) {
    // Check each TX channel
    for (int i = 0; i < 4; i++) {
        if (tx_instances[i] && dma_channel_get_irq0_status(tx_instances[i]->dma_chan)) {
            dma_channel_acknowledge_irq0(tx_instances[i]->dma_chan);
            tx_instances[i]->busy = false;
            if (tx_instances[i]->callback) {
                tx_instances[i]->callback(tx_instances[i]->callback_data);
            }
        }
    }
    
    // Check each RX channel
    for (int i = 0; i < 4; i++) {
        if (rx_instances[i] && dma_channel_get_irq0_status(rx_instances[i]->dma_chan)) {
            dma_channel_acknowledge_irq0(rx_instances[i]->dma_chan);
            rx_instances[i]->busy = false;
            if (rx_instances[i]->callback) {
                rx_instances[i]->callback(rx_instances[i]->callback_data);
            }
        }
    }
}

static void register_tx_instance(pio_spi_dma_tx_inst_t *inst) {
    for (int i = 0; i < 4; i++) {
        if (tx_instances[i] == NULL) {
            tx_instances[i] = inst;
            return;
        }
    }
}

static void unregister_tx_instance(pio_spi_dma_tx_inst_t *inst) {
    for (int i = 0; i < 4; i++) {
        if (tx_instances[i] == inst) {
            tx_instances[i] = NULL;
            return;
        }
    }
}

static void register_rx_instance(pio_spi_dma_rx_inst_t *inst) {
    for (int i = 0; i < 4; i++) {
        if (rx_instances[i] == NULL) {
            rx_instances[i] = inst;
            return;
        }
    }
}

static void unregister_rx_instance(pio_spi_dma_rx_inst_t *inst) {
    for (int i = 0; i < 4; i++) {
        if (rx_instances[i] == inst) {
            rx_instances[i] = NULL;
            return;
        }
    }
}

static bool irq_installed = false;

static void ensure_irq_handler(void) {
    if (!irq_installed) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
        irq_set_enabled(DMA_IRQ_0, true);
        irq_installed = true;
    }
}

// ============================================================================
// TX Implementation
// ============================================================================

pio_spi_dma_tx_inst_t pio_spi_dma_tx_init(PIO pio, uint sm,
                                           uint pin_clk, uint pin_data,
                                           float freq_hz) {
    pio_spi_dma_tx_inst_t inst = {
        .pio = pio,
        .sm = sm,
        .pio_offset = 0,
        .dma_chan = -1,
        .busy = false,
        .callback = NULL,
        .callback_data = NULL
    };
    
    // Load PIO program
    inst.pio_offset = pio_add_program(pio, &spi_tx_cs_program);
    spi_tx_cs_program_init(pio, sm, inst.pio_offset, pin_clk, pin_data, freq_hz);
    
    // Claim DMA channel
    inst.dma_chan = dma_claim_unused_channel(true);
    if (inst.dma_chan < 0) {
        return inst;  // Failed
    }
    
    // Configure DMA channel
    dma_channel_config c = dma_channel_get_default_config(inst.dma_chan);
    
    // Transfer 8 bits at a time
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    
    // Increment read address (source buffer), don't increment write (PIO FIFO)
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    
    // Pace transfers based on PIO TX FIFO
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));  // true = TX
    
    // Configure but don't start
    dma_channel_configure(
        inst.dma_chan,
        &c,
        &pio->txf[sm],      // Write to PIO TX FIFO
        NULL,               // Read address set later
        0,                  // Transfer count set later
        false               // Don't start yet
    );
    
    // Set up IRQ
    ensure_irq_handler();
    dma_channel_set_irq0_enabled(inst.dma_chan, true);
    
    register_tx_instance(&inst);
    
    return inst;
}

void pio_spi_dma_tx_start(pio_spi_dma_tx_inst_t *inst, const uint8_t *data, size_t len) {
    if (len == 0) return;
    
    inst->busy = true;
    
    // Set source and count, then start
    dma_channel_set_read_addr(inst->dma_chan, data, false);
    dma_channel_set_trans_count(inst->dma_chan, len, true);  // true = start
}

void pio_spi_dma_tx_wait(pio_spi_dma_tx_inst_t *inst) {
    dma_channel_wait_for_finish_blocking(inst->dma_chan);
    
    // Also wait for PIO FIFO to drain (DMA done doesn't mean PIO done)
    while (!pio_sm_is_tx_fifo_empty(inst->pio, inst->sm)) {
        tight_loop_contents();
    }
    
    inst->busy = false;
}

void pio_spi_dma_tx_blocking(pio_spi_dma_tx_inst_t *inst, const uint8_t *data, size_t len) {
    pio_spi_dma_tx_start(inst, data, len);
    pio_spi_dma_tx_wait(inst);
}

void pio_spi_dma_tx_set_callback(pio_spi_dma_tx_inst_t *inst,
                                  pio_spi_dma_callback_t callback,
                                  void *user_data) {
    inst->callback = callback;
    inst->callback_data = user_data;
}

void pio_spi_dma_tx_abort(pio_spi_dma_tx_inst_t *inst) {
    dma_channel_abort(inst->dma_chan);
    inst->busy = false;
}

void pio_spi_dma_tx_deinit(pio_spi_dma_tx_inst_t *inst) {
    // Abort any ongoing transfer
    pio_spi_dma_tx_abort(inst);
    
    // Disable IRQ for this channel
    dma_channel_set_irq0_enabled(inst->dma_chan, false);
    
    // Release DMA channel
    dma_channel_unclaim(inst->dma_chan);
    
    // Disable PIO SM
    pio_sm_set_enabled(inst->pio, inst->sm, false);
    pio_remove_program(inst->pio, &spi_tx_cs_program, inst->pio_offset);
    
    unregister_tx_instance(inst);
    
    inst->dma_chan = -1;
}

// ============================================================================
// RX Implementation
// ============================================================================

pio_spi_dma_rx_inst_t pio_spi_dma_rx_init(PIO pio, uint sm, uint pin_cs) {
    pio_spi_dma_rx_inst_t inst = {
        .pio = pio,
        .sm = sm,
        .pio_offset = 0,
        .dma_chan = -1,
        .busy = false,
        .callback = NULL,
        .callback_data = NULL
    };
    
    // Load PIO program
    inst.pio_offset = pio_add_program(pio, &spi_rx_cs_program);
    spi_rx_cs_program_init(pio, sm, inst.pio_offset, pin_cs);
    
    // Claim DMA channel
    inst.dma_chan = dma_claim_unused_channel(true);
    if (inst.dma_chan < 0) {
        return inst;  // Failed
    }
    
    // Configure DMA channel
    dma_channel_config c = dma_channel_get_default_config(inst.dma_chan);
    
    // Transfer 8 bits at a time
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    
    // Don't increment read (PIO FIFO), increment write (dest buffer)
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    
    // Pace transfers based on PIO RX FIFO
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));  // false = RX
    
    // Configure but don't start
    dma_channel_configure(
        inst.dma_chan,
        &c,
        NULL,                           // Write address set later
        &pio->rxf[sm],                  // Read from PIO RX FIFO
        0,                              // Transfer count set later
        false                           // Don't start yet
    );
    
    // Set up IRQ
    ensure_irq_handler();
    dma_channel_set_irq0_enabled(inst.dma_chan, true);
    
    register_rx_instance(&inst);
    
    return inst;
}

void pio_spi_dma_rx_start(pio_spi_dma_rx_inst_t *inst, uint8_t *data, size_t len) {
    if (len == 0) return;
    
    inst->busy = true;
    
    // Set destination and count, then start
    dma_channel_set_write_addr(inst->dma_chan, data, false);
    dma_channel_set_trans_count(inst->dma_chan, len, true);  // true = start
}

void pio_spi_dma_rx_wait(pio_spi_dma_rx_inst_t *inst) {
    dma_channel_wait_for_finish_blocking(inst->dma_chan);
    inst->busy = false;
}

void pio_spi_dma_rx_blocking(pio_spi_dma_rx_inst_t *inst, uint8_t *data, size_t len) {
    pio_spi_dma_rx_start(inst, data, len);
    pio_spi_dma_rx_wait(inst);
}

void pio_spi_dma_rx_set_callback(pio_spi_dma_rx_inst_t *inst,
                                  pio_spi_dma_callback_t callback,
                                  void *user_data) {
    inst->callback = callback;
    inst->callback_data = user_data;
}

void pio_spi_dma_rx_abort(pio_spi_dma_rx_inst_t *inst) {
    dma_channel_abort(inst->dma_chan);
    inst->busy = false;
}

void pio_spi_dma_rx_flush(pio_spi_dma_rx_inst_t *inst) {
    // Drain FIFO manually
    while (!pio_sm_is_rx_fifo_empty(inst->pio, inst->sm)) {
        (void)pio_sm_get(inst->pio, inst->sm);
    }
}

void pio_spi_dma_rx_deinit(pio_spi_dma_rx_inst_t *inst) {
    // Abort any ongoing transfer
    pio_spi_dma_rx_abort(inst);
    
    // Disable IRQ for this channel
    dma_channel_set_irq0_enabled(inst->dma_chan, false);
    
    // Release DMA channel
    dma_channel_unclaim(inst->dma_chan);
    
    // Disable PIO SM
    pio_sm_set_enabled(inst->pio, inst->sm, false);
    pio_remove_program(inst->pio, &spi_rx_cs_program, inst->pio_offset);
    
    unregister_rx_instance(inst);
    
    inst->dma_chan = -1;
}
