/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-

triggerbox - trigger box for arduino
====================================

This file contains the source code for the firmware that runs on the
Arduino device. There are two notable things that happen here.


Regular hardware trigger pulse generation
-----------------------------------------

We need to send trigger pulses at a regular interval.

Clock synchronization support
-----------------------------

See the file synchronization.md for information about the theory of
operation. In this firmware, we need to respond to clock requests with
our current clock value. We use the device's timer1 as our official
clock.

 */

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <UDEV.h>

#define WITH_AOUT
#ifdef WITH_AOUT

// ----------------------- start MCP4822 library

/* This MCP4822 code is included in this file so the whole firmware
   can be distributed as a single Arduino file without having to
   install libraries. It was originally written by Will Dickson, IO
   Rodeo Inc. It was downloaded from
   https://bitbucket.org/iorodeo/iorodeo_arduino_libs . It is licensed
   under the Apache 2.0 licence.

*/

#define MCP4822_NUMCHAN 2

enum MCP4822_DAC_CHAN {MCP4822_DAC_A, MCP4822_DAC_B};

class MCP4822 {
private:
    int cs;
    int ldac;
    int gain[MCP4822_NUMCHAN];
    int getCmdWrd(int dac, int value);
public:
    MCP4822();
    MCP4822(int csPin, int ldacPin);
    void setValue(int dac, int value);
    void setValue_A(int value);
    void setValue_B(int value);
    void setValue_AB(int value_A, int value_B);
    void setGain2X(int dac);
    void setGain2X_A();
    void setGain2X_B();
    void setGain2X_AB();
    void setGain1X(int dac);
    void setGain1X_A();
    void setGain1X_B();
    void setGain1X_AB();
    void off_AB();
    void off_A();
    void off_B();
    void off(int dac);
};
// ----------------------------------------------------------------------------
// max4822.cpp
//
// Provides an SPI based interface to the MCP4822 dual voltage output digital
// to analog converter.
//
// Author: Will Dickson, IO Rodeo Inc.
// ----------------------------------------------------------------------------

#define ACTIVE      0b0001000000000000
#define SHUTDOWN    0b0000000000000000
#define GAIN_2X     0b0000000000000000
#define GAIN_1X     0b0010000000000000
#define DAC_A_WRITE 0b0000000000000000
#define DAC_B_WRITE 0b1000000000000000

MCP4822::MCP4822() {
}

// ----------------------------------------------------------------------------
// MCP4822::MCP4822
//
// Constructor
// ----------------------------------------------------------------------------
MCP4822::MCP4822(int csPin, int ldacPin) {
    // Configure chip select and latch pins
    cs = csPin;
    ldac = ldacPin;
    pinMode(cs,OUTPUT);
    pinMode(ldac,OUTPUT);
    digitalWrite(cs,HIGH);
    digitalWrite(ldac,HIGH);
    // Set to default configuration
    setGain2X_AB();
}

// ---------------------------------------------------------------------------
// MCP4822::getCmdWrd
//
// Gets command work for writing value to given channel
// ---------------------------------------------------------------------------
int MCP4822::getCmdWrd(int dac, int value) {
    int cmdWrd = 0;
    switch (dac) {
        case MCP4822_DAC_A:
            cmdWrd = DAC_A_WRITE;
            break;
        case MCP4822_DAC_B:
            cmdWrd = DAC_B_WRITE;
            break;
        default:
            return 0;
    }
    cmdWrd |= gain[dac];
    cmdWrd |= ACTIVE;
    cmdWrd |= (0b0000111111111111 & value);
    return cmdWrd;
}

// ----------------------------------------------------------------------------
// MCP4822::setValue
//
// Set the output value of the given dac channel
// ----------------------------------------------------------------------------
void MCP4822::setValue(int dac, int value) {
    int cmdWrd;
    uint8_t byte0;
    uint8_t byte1;

    // Enable SPI communications
    digitalWrite(cs,LOW);
    // Send command word
    cmdWrd = getCmdWrd(dac,value);
    byte0 = cmdWrd >> 8;
    byte1 = cmdWrd & 0b0000000011111111;
    SPI.transfer(byte0);
    SPI.transfer(byte1);
    // Disable SPI communications
    digitalWrite(cs,HIGH);
    // Latch value
    digitalWrite(ldac,LOW);
    digitalWrite(ldac,HIGH);
}

// ---------------------------------------------------------------------------
// MCP4822::setValue_A
//
// Set the output value of dac A to the given value
// ---------------------------------------------------------------------------
void MCP4822::setValue_A(int value) {
    setValue(MCP4822_DAC_A, value);
}

// ----------------------------------------------------------------------------
// MCP4822::setValue_B
//
// Set the output value of dac B to the given value
// ----------------------------------------------------------------------------
void MCP4822::setValue_B(int value) {
    setValue(MCP4822_DAC_B,value);
}

// ---------------------------------------------------------------------------
// MCP4822::setValue_AB
//
// Set the output value of dac A and B to the given values. Latch them in at
// the same time.
// ---------------------------------------------------------------------------
void MCP4822::setValue_AB(int value_A, int value_B) {
    int cmdWrd;
    uint8_t byte0;
    uint8_t byte1;
    // Enable SPI communications and send command word
    digitalWrite(cs,LOW);
    // Send command word for DAC A
    cmdWrd = getCmdWrd(MCP4822_DAC_A,value_A);
    byte0 = cmdWrd >> 8;
    byte1 = cmdWrd & 0b0000000011111111;
    SPI.transfer(byte0);
    SPI.transfer(byte1);
    // Toggle CS
    digitalWrite(cs,HIGH);
    digitalWrite(cs,LOW);
    // Send command word for DAC A
    cmdWrd = getCmdWrd(MCP4822_DAC_B,value_B);
    byte0 = cmdWrd >> 8;
    byte1 = cmdWrd & 0b0000000011111111;
    SPI.transfer(byte0);
    SPI.transfer(byte1);
    // Disable SPI communications
    digitalWrite(cs,HIGH);
    // Latch value
    digitalWrite(ldac,LOW);
    digitalWrite(ldac,HIGH);

}

// ---------------------------------------------------------------------------
// MCP4822::setGain2X
//
// Set the gain of the given channel to 2X
// ---------------------------------------------------------------------------
void MCP4822::setGain2X(int dac) {
    if ((dac == MCP4822_DAC_A) || (dac == MCP4822_DAC_B)) {
        gain[dac] = GAIN_2X;
    }
}

// ----------------------------------------------------------------------------
// MCP4822::setGain2X_A
//
// Set the gain of dac A to 2X
// ----------------------------------------------------------------------------
void MCP4822::setGain2X_A() {
    setGain2X(MCP4822_DAC_A);
}

// ----------------------------------------------------------------------------
// MCP4822::setGain2X_B
//
// Set the gain of dac B to 2X
// ----------------------------------------------------------------------------
void MCP4822::setGain2X_B() {
    setGain2X(MCP4822_DAC_B);
}

// ----------------------------------------------------------------------------
// MCP4822::setGain2X_AB
//
// Set the gain of dac A and B to 2X
// ----------------------------------------------------------------------------
void MCP4822::setGain2X_AB() {
    setGain2X_A();
    setGain2X_B();
}

// ---------------------------------------------------------------------------
// MCP4822::setGain1X
//
// Set the gain of the given channel to 1X
// ----------------------------------------------------------------------------
void MCP4822::setGain1X(int dac) {
    if ((dac == MCP4822_DAC_A) || (dac == MCP4822_DAC_B)) {
        gain[dac] = GAIN_1X;
    }
}

// ----------------------------------------------------------------------------
// MCP4822::setGain1X_A
//
// Set the gain of dac A to 1X
// ----------------------------------------------------------------------------
void MCP4822::setGain1X_A() {
    setGain1X(MCP4822_DAC_A);
}

// ----------------------------------------------------------------------------
// MCP4822::setGain1X_B
//
// Set the gain of dac B to 1X
// ----------------------------------------------------------------------------
void MCP4822::setGain1X_B() {
    setGain1X(MCP4822_DAC_B);
}

// ----------------------------------------------------------------------------
// MCP4822::setGain1X_AB
//
// Set the gain of dac A and B to 1X
// ----------------------------------------------------------------------------
void MCP4822::setGain1X_AB() {
    setGain1X_A();
    setGain1X_B();
}

// ----------------------------------------------------------------------------
// MCP4822::off
//
// Turn off the given dac
// ----------------------------------------------------------------------------
void MCP4822::off(int dac) {
    int cmdWrd = 0;
    uint8_t byte0;
    uint8_t byte1;

    // Create shutdown cmd word.
    switch (dac) {
        case MCP4822_DAC_A:
            cmdWrd = DAC_A_WRITE;
            break;
        case MCP4822_DAC_B:
            cmdWrd = DAC_B_WRITE;
            break;
        default:
            return;
    }
    cmdWrd |= SHUTDOWN;
    // Enable SPI communications
    digitalWrite(cs,LOW);
    // Send command word
    byte0 = cmdWrd >> 8;
    byte1 = cmdWrd & 0b0000000011111111;
    SPI.transfer(byte0);
    SPI.transfer(byte1);
    // Disable SPI communications
    digitalWrite(cs,HIGH);
}

// ----------------------------------------------------------------------------
// MCP4822::off_A
//
// Turn off dac A
// ----------------------------------------------------------------------------
void MCP4822::off_A() {
    off(MCP4822_DAC_A);
}

// ----------------------------------------------------------------------------
// MCP4822::off_B
//
// Turn off dac B
// ----------------------------------------------------------------------------
void MCP4822::off_B() {
    off(MCP4822_DAC_B);
}

// ----------------------------------------------------------------------------
// MCP4822::off_AB
//
// Turn off dac A and B
// ----------------------------------------------------------------------------
void MCP4822::off_AB() {
    off_A();
    off_B();
}

#endif // WITH_AOUT

// ----------------------- end MCP4822 library

typedef uint32_t pulsenumber_dtype; /* 2**32 @100Hz = 497 days */

struct timed_sample {
    uint8_t value; /* value of arbitrary data */
    pulsenumber_dtype pulsenumber;
    uint16_t ticks;
};

// Pin numbers -----------------------------------------------------------------
const unsigned short TrigPin = 9;
const unsigned short LEDPin = 2;
#ifdef WITH_AOUT
const unsigned short AOUT_CS = 10;
const unsigned short AOUT_LDAC = 7;
#endif

// Global variables ------------------------------------------------------------
volatile pulsenumber_dtype pulsenumber=0;
    // ---- set TCCR1B ----------
    // high bits = 0,0,0
    // WGM13, WGM12 = 1,1
    // CS1 = 0,1,0 (starts timer1 CS=8) (clock select)
uint8_t tccr1b_when_running = 0x18; //stopped. started in setup()
#ifdef WITH_AOUT
MCP4822 analogOut = MCP4822(AOUT_CS,AOUT_LDAC);
#endif

UDEV udev(Serial);

// Interrupt service routine for timer1 compare -------------------------------
ISR(TIMER1_COMPA_vect)
{
    pulsenumber++;
}

// Setup timer1 ----------------------------------------------------------------
void setup_timer1(uint8_t tccr1b, uint16_t icr1) {
    tccr1b_when_running = tccr1b;

    cli();

    // Set output compare to generate trigger pulse width
    OCR1A = 0x03e8;

    // Set TOP
    ICR1 = icr1;

    // ---- set TCCR1A ----------
    // set Compare Output Mode for Fast PWM, with TOP in ICR1
    // COM1A1:0 = 1,0 clear OC1A on compare match
    // COM1B1:0 = 1,0 clear OC1B on compare match
    // WGM11, WGM10 = 1,0
    TCCR1A = 0xAA;
    TCCR1B = tccr1b;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

// Standard arduino setup function ---------------------------------------------
void setup() {
    udev.begin();
    udev.serial_handshake();   //blocks for 5 seconds by default

    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.begin();

#ifdef WITH_AOUT
    analogOut.setGain2X_AB();
#endif

    pinMode(LEDPin, OUTPUT);
    pinMode(TrigPin, OUTPUT);

    // start serial port at 115200 bps:
    Serial.begin(115200);

    digitalWrite(LEDPin, HIGH);

    // start at 25fps
    setup_timer1(0x1B, 0x2710);
}

// Send data with our simple protocol to the host computer ---------------------
static char Serial_read_blocking() {
    while (Serial.available() == 0) {
        delay(10);
    }
    return Serial.read();
}

static inline void send_data_string(String* s, const char header) {
    uint8_t N = s->length();

    Serial.write(header);
    Serial.write(N); // payload size
    uint8_t chksum=0;
    char next_byte;
    for (uint8_t i=0; i< N; i++) {
        next_byte = s->charAt(i);
        chksum += next_byte;
        Serial.write(next_byte);
    }
    Serial.write(chksum);
}

static inline void send_data(const struct timed_sample samp, const char header) {

    /* This commented-out version does the same as below but with more
       overhead. Nevertheless, it can be used to verify the
       correctness of the send_data_string() function.

    const char * buf;
    buf = (const char*)&(samp);

    String out_buf="";

    for (uint8_t i=0; i< sizeof(timed_sample); i++) {
        out_buf += buf[i];
    }
    send_data_string(&out_buf, header);
    */

    const char * buf;
    Serial.write(header);
    Serial.write((char)sizeof(struct timed_sample)); // payload size
    buf = (const char*)&(samp);
    uint8_t chksum=0;
    for (uint8_t i=0; i< sizeof(struct timed_sample); i++) {
        chksum += buf[i];
        Serial.write(buf[i]);
    }
    Serial.write(chksum);
}

// Standard Arduino loop function. This gets repeatedly called at high rate ----
void loop() {
    if (Serial.available() >= 2) {
        static char cmd;
        static char value;

        cmd = Serial.read();
        value = Serial.read();

        if (cmd=='P') {
            // timestamp query
            static struct timed_sample timestamp_request;

            timestamp_request.value = value;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                timestamp_request.pulsenumber = pulsenumber;
                timestamp_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(timestamp_request,'P');
        } else if (cmd=='V') {
            // version request
            static struct timed_sample version_request;

            version_request.value = 14;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                version_request.pulsenumber = pulsenumber;
                version_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(version_request,'V');

            digitalWrite(LEDPin, LOW);

        } else if (cmd=='S') {
            // synchronization

            if (value=='0') {
                // stop clock, reset pulsenumber
                TCCR1B = 0x18;
                TCNT1 = 0;
                pulsenumber = 0;
                digitalWrite(LEDPin, HIGH);
            } else if (value=='1') {
                // start clock
                TCCR1B = tccr1b_when_running;
                digitalWrite(LEDPin, LOW);
            } else if (value=='2') {
                // stop clock
                TCCR1B = 0x18;
                digitalWrite(LEDPin, HIGH);
            }

        } else if (cmd=='T') {
            // TOP value
            uint8_t value0, value1;
            char prescaler_key;
            uint16_t new_icr1;

            if (value=='=') {
                value0 = Serial_read_blocking();
                value1 = Serial_read_blocking();
                prescaler_key = Serial_read_blocking();
                new_icr1 = ((uint16_t)value1 << 8) + value0;
                ICR1 = new_icr1;
                if (prescaler_key=='1') {
                    // prescaler = 8
                    tccr1b_when_running = 0x1A;
                } else if (prescaler_key=='2') {
                    // prescaler = 64
                    tccr1b_when_running = 0x1B;
                }
            }

        } else if (cmd=='O') {
            // AOUT values
            uint8_t aout0_0, aout0_1,   aout1_0, aout1_1;
            uint8_t aout_sequence;
            int aout0, aout1;

            if (value=='=') {
                aout0_0 = Serial_read_blocking();
                aout0_1 = Serial_read_blocking();
                aout0 = ((int)aout0_1 << 8) + aout0_0;

                aout1_0 = Serial_read_blocking();
                aout1_1 = Serial_read_blocking();
                aout1 = ((int)aout1_1 << 8) + aout1_0;

#ifdef WITH_AOUT
                analogOut.setValue_AB(aout0, aout1);
#endif
                aout_sequence = Serial_read_blocking();


            static struct timed_sample aout_confirm;

            aout_confirm.value = aout_sequence;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                aout_confirm.pulsenumber = pulsenumber;
                aout_confirm.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(aout_confirm,'O');

            }

        } else if (cmd=='N') {
            udev.process(cmd, value);
        }

    }

}
