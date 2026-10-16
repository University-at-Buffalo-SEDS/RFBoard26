#ifndef UART_DMA_RX_H
#define UART_DMA_RX_H
#include <stddef.h>
#include <stdint.h>

/* Circular ReceiveToIdle positions are cumulative, not chunk lengths.
 * Keep HT enabled so every full DMA lap has an observable midpoint. Retain N
 * as the end position: TC followed by IDLE at N must not deliver a lap twice. */
static inline void uart_dma_rx_consume(const uint8_t *buffer, uint16_t capacity,
                                      uint16_t position, uint16_t *previous,
                                      void (*consume)(const uint8_t *, uint16_t))
{
    if (buffer == NULL || previous == NULL || consume == NULL ||
        position == 0U || position > capacity) return;
    const uint16_t start = *previous;
    if (position == start) return;
    if (position < start) {
        if (start < capacity) consume(buffer + start, capacity - start);
        consume(buffer, position);
    } else {
        consume(buffer + start, position - start);
    }
    *previous = position;
}
#endif
