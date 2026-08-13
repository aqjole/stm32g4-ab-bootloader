/*
 * g4b_log.h - tiny UART logging, shared declaration.
 *
 * Defined separately in each project's main.c (each has its own huart2).
 * Deliberately not newlib printf: nano.specs vsnprintf into a fixed buffer,
 * then HAL_UART_Transmit. The bootloader has 16 KB total and no business
 * pulling in stdio's file machinery.
 */

#ifndef G4B_LOG_H
#define G4B_LOG_H

#include <stdint.h>

void g4b_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Called from the naked HardFault shim with a pointer to the stacked frame:
   frame[0..7] = r0 r1 r2 r3 r12 lr pc xpsr */
void g4b_fault_report(uint32_t *frame);

#endif /* G4B_LOG_H */
