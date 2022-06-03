#include <Arduino.h>
#include <CrsfSerial.h>
#include <median.h>
#include <Servo.h>
#include "target.h"
#include "pins_arduino.h"
#include <PicoLed.hpp>

#include "config.h"

//default design has 8MB flash
#define PICO_FLASH_SIZE_BYTES 8 * 1024 * 1024

#define NUM_OUTPUTS 12

// Configuration
// Map input CRSF channels (1-based, up to 16 for CRSF, 12 for ELRS) to outputs 1-8
// use a negative number to invert the signal (i.e. +100% becomes -100%)
constexpr int OUTPUT_MAP[NUM_OUTPUTS] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
// The failsafe action for each channel (fsaNoPulses, fsaHold, or microseconds)
constexpr int OUTPUT_FAILSAFE[NUM_OUTPUTS] = {
    1500, 1500, 988, 1500,                  // ch1-ch4
    fsaHold, fsaHold, fsaHold, fsaNoPulses, // ch5-ch8
    fsaHold, fsaHold, fsaHold, fsaHold,     // ch9-ch12
};

// Define the pins used to output servo PWM, must use hardware PWM,
// and change HardwareTimer targets below if the timers change
constexpr int OUTPUT_PINS[NUM_OUTPUTS] = { OUTPUT_PIN_MAP };

#if !defined(PWM_FREQ_HZ)
    #define PWM_FREQ_HZ     50
#endif
#define VBAT_INTERVAL   500
#define VBAT_SMOOTH     5
// Scale used to calibrate or change to CRSF standard 0.1 scale
#define VBAT_SCALE      1.0

SerialUART serial2(uart0, UART_INPUT_TX, UART_INPUT_RX, 0, 0);
static CrsfSerial crsf(serial2);

static int g_OutputsUs[NUM_OUTPUTS];
static struct tagConnectionState {
    uint32_t lastVbatRead;
    MedianAvgFilter<unsigned int, VBAT_SMOOTH>vbatSmooth;
    unsigned int vbatValue;

    char serialInBuff[64];
    uint8_t serialInBuffLen;
    bool serialEcho;
    Servo* pwm[12];
} g_State;

auto led = PicoLed::addLeds<PicoLed::WS2812B>(pio0, 0, DPIN_LED, 4, PicoLed::FORMAT_GRB);

static void crsfShiftyByte(uint8_t b)
{
    // A shifty byte is usually just log messages from ELRS
    Serial.write(b);
}

static void servoSetUs(unsigned int servo, int usec)
{
  static int write_resolution = 10;
  static int maxRes = 1024;
  static int pin;
  static int val;
  pin = OUTPUT_PINS[servo];
  val = map(usec, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MAX, 0, maxRes);
  float percent = (float)val/(float)((1 << write_resolution)-1);

  if (g_State.pwm[pin] == NULL) {
      Servo pwm;
      
      g_State.pwm[pin] = &pwm;
      g_State.pwm[pin]->attach(pin, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, OUTPUT_FAILSAFE[pin]);
  }
  
  //mbed::PwmOut* pwm = digitalPinToPwm(pin);
  /*if (pwm == NULL) {
      pwm = new mbed::PwmOut(digitalPinToPinName(pin));
      digitalPinToPwm(pin) = pwm;
      #if defined(PWM_FREQ_HZ)
          pwm->period_ms(PWM_FREQ_HZ / 1000.0);
      #else
          pwm->period_ms(2); //500Hz // TODO: apply PWM_FREQ_HZ
      #endif
  }*/
  if (percent >= 0) {
      g_State.pwm[pin]->write(percent);
  } else {
      g_State.pwm[pin]->detach();
      delete g_State.pwm[pin];
      g_State.pwm[pin] = NULL;
  }
  g_OutputsUs[servo] = usec;
}

static void packetChannels()
{
    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        const int chInput = OUTPUT_MAP[out];
        int usOutput;
        if (chInput > 0)
            usOutput = crsf.getChannel(chInput);
        else
        {
            // if chInput is negative, invert the channel output
            usOutput = crsf.getChannel(-chInput);
            // (1500 - usOutput) + 1500
            usOutput = 3000 - usOutput;
        }
        servoSetUs(out, usOutput);
    }

    // for (unsigned int ch=0; ch<4; ++ch)
    // {
    //     Serial.write(ch < 10 ? '0' + ch : 'A' + ch - 10);
    //     Serial.write('=');
    //     Serial.print(crsf.getChannel(ch), DEC);
    //     Serial.write(' ');
    // }
    // Serial.println();
}

static void packetLinkStatistics(crsfLinkStatistics_t *link)
{
  //Serial.print(link->uplink_RSSI_1, DEC);
  //Serial.println("dBm");
}

static void crsfLinkUp()
{
    led.setPixelColor(1, PicoLed::RGB(0, 255, 0));
    
    //digitalWrite(DPIN_LED, HIGH ^ LED_INVERTED);
}

static void crsfLinkDown()
{
    led.setPixelColor(1, PicoLed::RGB(255, 255, 0));
    
    // digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);

    // Perform the failsafe action
    for (unsigned int out=0; out<NUM_OUTPUTS; ++out)
    {
        if (OUTPUT_FAILSAFE[out] == fsaNoPulses)
            servoSetUs(out, 0);
        else if (OUTPUT_FAILSAFE[out] != fsaHold)
            servoSetUs(out, OUTPUT_FAILSAFE[out]);
        // else fsaHold does nothing, keep the same value
    }
}

static void checkVbatt()
{
    if (millis() - g_State.lastVbatRead < (VBAT_INTERVAL / VBAT_SMOOTH))
        return;
    g_State.lastVbatRead = millis();

    unsigned int idx = g_State.vbatSmooth.add(analogRead(APIN_VBAT));
    if (idx != 0)
        return;

    unsigned int adc = g_State.vbatSmooth;
    g_State.vbatValue = 330U * adc * (VBAT_R1 + VBAT_R2) / VBAT_R2 / ((1 << 12) - 1);

    crsf_sensor_battery_t crsfbatt = { 0 };
    uint16_t scaledVoltage = g_State.vbatValue * VBAT_SCALE;
    // Values are MSB first (BigEndian)
    crsfbatt.voltage = htobe16(scaledVoltage);
    crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfbatt, sizeof(crsfbatt));

    //Serial.print("ADC="); Serial.print(adc, DEC);
    //Serial.print(" "); Serial.print(g_State.vbatValue, DEC); Serial.println("V");
}

static bool handleSerialCommand(char *cmd)
{
    // Fake a CRSF RX on UART6
    bool prompt = true;
    if (strcmp(cmd, "#") == 0)
    {
        Serial.println("Fake CLI Mode, type 'exit' or 'help' to do nothing\r\n");
        g_State.serialEcho = true;
    }

    else if (strcmp(cmd, "serial") == 0)
        Serial.println("serial 1 64 0 0 0 0\r\n");

    else if (strcmp(cmd, "get serialrx_provider") == 0)
        Serial.println("serialrx_provider = CRSF\r\n");

    else if (strcmp(cmd, "get serialrx_inverted") == 0)
        Serial.println("serialrx_inverted = OFF\r\n");

    else if (strcmp(cmd, "get serialrx_halfduplex") == 0)
        Serial.println("serialrx_halfduplex = OFF\r\n");

    else if (strncmp(cmd, "serialpassthrough 1 ", 20) == 0)
    {
        Serial.println("Passthrough serial 1");
        // Force a reboot command since we want to send the reboot
        // at 420000 then switch to what the user wanted
        const uint8_t rebootcmd[] = { 0xEC,0x04,0x32,0x62,0x6c,0x0A };
        crsf.write(rebootcmd, sizeof(rebootcmd));

        unsigned int baud = atoi(cmd+20);
        crsf.setPassthroughMode(true, baud);
        g_State.serialEcho = false;
        return false;
    }

    else
        prompt = false;

    if (prompt)
        Serial.print("# ");

    return true;
}

static void checkSerialInPassthrough()
{
    static uint32_t lastData = 0;
    static bool LED = false;
    bool gotData = false;

    // Simple data passthrough from in to crsf
    unsigned int avail;
    while ((avail = Serial.available()) != 0)
    {
        uint8_t buf[16];
        avail = Serial.readBytes((char *)buf, min(sizeof(buf), avail));
        crsf.write(buf, avail);
        digitalWrite(DPIN_LED, LED);
        LED = !LED;
        gotData = true;
    }

    // If longer than X seconds since last data, switch out of passthrough
    if (gotData || !lastData)
        lastData = millis();
    else if (millis() - lastData > 3000)
    {
        lastData = 0;
        digitalWrite(DPIN_LED, HIGH ^ LED_INVERTED);
        delay(250);
        digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
        crsf.setPassthroughMode(false);
    }
}

static void checkSerialInNormal()
{
    while (Serial.available())
    {
        led.setPixelColor(1, PicoLed::RGB(0, 0, 255));

        char c = Serial.read();
        if (g_State.serialEcho && c != '\n')
            Serial.write(c);

        if (c == '\r' || c == '\n')
        {
            if (g_State.serialInBuffLen != 0)
            {
                Serial.write('\n');
                Serial.flush();
                g_State.serialInBuff[g_State.serialInBuffLen] = '\0';
                handleSerialCommand(g_State.serialInBuff);
                g_State.serialInBuffLen = 0;
            }
        }
        else
        {
            g_State.serialInBuff[g_State.serialInBuffLen++] = c;
            // if the buffer fills without getting a newline, just reset
            if (g_State.serialInBuffLen >= sizeof(g_State.serialInBuff))
                g_State.serialInBuffLen = 0;
        }
    }  /* while Serial */
}

static void checkSerialIn()
{
    if (crsf.getPassthroughMode())
        checkSerialInPassthrough();
    else
        checkSerialInNormal();
}

static void setupCrsf()
{
    crsf.onLinkUp = &crsfLinkUp;
    crsf.onLinkDown = &crsfLinkDown;
    crsf.onShiftyByte = &crsfShiftyByte;
    crsf.onPacketChannels = &packetChannels;
    crsf.onPacketLinkStatistics = packetLinkStatistics;
    //crsf.setPassthroughMode(true);
}

static void setupGpio()
{
    //pinMode(DPIN_LED, OUTPUT);
    //digitalWrite(DPIN_LED, LOW ^ LED_INVERTED);
    analogReadResolution(12);

    // The servo outputs are initialized when the
    // first channels packet comes in and sets the PWM
    // output value, to prevent them from jerking around
    // on startup
}

void defaultLed() {
    led.clear();

    led.setPixelColor(0, PicoLed::RGB(255,255,0));
    led.setPixelColor(1, PicoLed::RGB(255,0,0));
    led.setPixelColor(2, PicoLed::RGB(0,0,0));
    led.setPixelColor(3, PicoLed::RGB(0,0,0));
}

void setup() {
    Serial.begin(115200);

    led.setDrawMode(PicoLed::DrawMode::MODE_SET);

    defaultLed();

    led.show();

    setupGpio();
    setupCrsf();
}

void loop() {
    crsf.loop();

    checkVbatt();
    checkSerialIn();

    led.show();
}