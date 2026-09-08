#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "./frmeter.pio.h"
#include <limits.h>
#include "TM1637.h"

// 4-digit TM1637 seven-segment display driver instance.
TM1637 TM;

#define ZERO_LED_BUILTIN           8
#define FR_INPUT_PIN               15
#define RANGE_SEL_PIN              14  // Pulled-up input: HIGH = direct, LOW = external /64 prescaler in use.
#define PRESCALER_RATIO            64

// Set from IRQ when a new measurement is available for loop().
volatile bool updated = false;
// Simple diagnostics counter: number of handled PIO IRQs.
uint32_t cntr=0;
// Raw down-counter value captured by PIO state machine 1.
volatile uint32_t pulseCount;
// Frequency in MHz shown on display.
float fr=0;
// Scratch buffer kept for optional debug formatting.
char buffer[32];

// Timestamp of the last valid measurement update.
unsigned long lastupdated = 0;

// PIO IRQ handler:
// - Reads captured counter value from SM1 RX FIFO.
// - Reloads gate/counter state machines for next sampling window.
// - Signals main loop that a new value is ready for display.
void pio_irq_handler() {
  if (pio_interrupt_get(pio0, 0)) {
    pio_interrupt_clear(pio0, 0);

    // Only read when RX FIFO has data; prevents ISR lockup when input signal is missing.
    if (!pio_sm_is_rx_fifo_empty(pio0, 1)) {
      pulseCount = pio_sm_get(pio0, 1);
      updated = true;
    }

    pio_sm_put(pio0, 0, 12500000);
    pio_sm_put(pio0, 1, UINT32_MAX);
    cntr++;
  }
}

void setup() { 
    // Let peripherals settle after boot.
    sleep_ms(1000);

    // TM1637 wiring: CLK pin 2, DIO pin 3, 4 digits total.
    TM.begin(2, 3, 4); // clock pin, data pin, number of digits
    TM.setBrightness(6); // brightness 0..7
    TM.displayInt(1234);
    
    // Load and initialize gate-time PIO program on state machine 0.
    uint offset = pio_add_program(pio0, &gate_program);
    sprintf(buffer, "%u", offset);
    init_gate(pio0, 0, offset, ZERO_LED_BUILTIN, 12500000);

    // Load and initialize frequency counter PIO program on state machine 1.
    offset = pio_add_program(pio0, &fin_cntr_program);
    init_fin_cntr(pio0, 1, offset, FR_INPUT_PIN, ZERO_LED_BUILTIN);
    gpio_pull_down(FR_INPUT_PIN);

    // Range selector: HIGH (default via pull-up) = direct, LOW = /64 prescaler in signal path.
    pinMode(RANGE_SEL_PIN, INPUT_PULLUP);

    // Route PIO interrupt 0 to CPU IRQ handler.
    pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);
    
    sleep_ms(1000);
    TM.displayFloat(0.0, 3);
    lastupdated = millis();
}

void loop() {
    if (updated) {
        // Convert captured pulse count to MHz.
        // Gate time is 0.1 s, so 100000 counts correspond to 1 MHz.
        fr = ((float)(UINT32_MAX-pulseCount))/100000.0; // Convert frequency to Mhz as gate time is 0.1s

        // If range selector is grounded, an external /64 prescaler is in the signal path;
        // compensate by multiplying the measured value.
        if (digitalRead(RANGE_SEL_PIN) == LOW) {
            fr *= PRESCALER_RATIO;
        }

        // Dynamic decimal precision to keep meaningful resolution on 4 digits.
        if (fr < 10.0) { 
          TM.displayFloat(fr, 3);
        } else 
        if (fr < 100.0) {
          TM.displayFloat(fr, 2);
        } else {
          TM.displayFloat(fr, 1);
        }
        lastupdated = millis();
        updated = false;
    }

    // Signal loss watchdog: clear stale readings if updates stop arriving.
    if (millis() - lastupdated > 300) { // If no update for 300ms, show 0.000
        TM.displayFloat(0.0, 3);
    }
}
