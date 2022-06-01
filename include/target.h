#pragma once

#define DPIN_LED        LED_BUILTIN
#define LED_INVERTED    0
#define APIN_VBAT       p28
#define UART_INPUT_TX   8
#define UART_INPUT_RX   9
#define OUTPUT_PIN_MAP  p10, p11, p12, p13,  p18, p19, p20, p21

#if !defined(LED_INVERTED)
    #define LED_INVERTED    0
#endif

#if !defined(VBAT_R1) || !defined(VBAT_R2)
    // Resistor divider used on VBAT input, R1+R2 must be less than 3178
    #define VBAT_R1         820
    #define VBAT_R2         120
#endif