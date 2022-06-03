#pragma once

#define DPIN_LED        D27
#define LED_INVERTED    0
#define APIN_VBAT       D26
#define UART_INPUT_TX   D16
#define UART_INPUT_RX   D17
#define OUTPUT_PIN_MAP  D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11

#if !defined(LED_INVERTED)
    #define LED_INVERTED    0
#endif

#if !defined(VBAT_R1) || !defined(VBAT_R2)
    // Resistor divider used on VBAT input, R1+R2 must be less than 3178
    #define VBAT_R1         900
    #define VBAT_R2         100
#endif