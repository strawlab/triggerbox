/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-

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

typedef uint32_t pulsenumber_dtype; /* 2**32 @100Hz = 497 days */

struct timed_sample {
    uint8_t value; /* value of arbitrary data */
    pulsenumber_dtype pulsenumber;
    uint16_t ticks;
};

// Pin numbers -----------------------------------------------------------------
#define LEDPin 13

// Global variable for clock measurement ---------------------------------------
volatile pulsenumber_dtype pulsenumber=0;

// Interrupt service routine for timer1 compare -------------------------------
ISR(TIMER1_COMPA_vect)
{
    pulsenumber++;
}

// Setup timer1 ----------------------------------------------------------------
void setup_timer1() {
    cli();

    // Set output compare to generate trigger pulse width
    OCR1A = 0x03e8;

    // Set TOP
    ICR1 = 0x2710;

    // ---- set TCCR1A ----------
    // set Compare Output Mode for Fast PWM, with TOP in ICR1
    // COM1A1:0 = 1,0 clear OC1A on compare match
    // COM1B1:0 = 1,0 clear OC1B on compare match
    // WGM11, WGM10 = 1,0
    TCCR1A = 0xAA;

    // ---- set TCCR1B ----------
    // high bits = 0,0,0
    //WGM13, WGM12 = 1,1
    // CS1 = 0,1,0 (starts timer1 CS=8) (clock select)
    TCCR1B = 0x1A;

    TIMSK1 |= (1 << OCIE1A);

    sei();
}

// Standard arduino setup function ---------------------------------------------
void setup() {

    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, 0);

    pinMode(9, OUTPUT);

    // start serial port at 115200 bps:
    Serial.begin(115200);

    setup_timer1();
}

// Send data with our simple protocol to the host computer ---------------------
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

            version_request.value = 11;

            uint8_t SaveSREG_ = SREG;   // save interrupt flag
            cli(); // disable interrupts

                version_request.pulsenumber = pulsenumber;
                version_request.ticks = TCNT1;

            SREG = SaveSREG_; // restore interrupt flags
            send_data(version_request,'V');
        } else if (cmd=='S') {
            // synchronization

            if (value=='0') {
                // stop clock, reset pulsenumber
                TCCR1B = 0x18;
                pulsenumber = 0;
            } else if (value=='1') {
                // start clock
                TCCR1B = 0x1A;
            }

        } else if (cmd=='T') {
            // TOP value
            char value0, value1;
            uint16_t new_icr1;

            if (value=='=') {
                while (Serial.available() == 0) {
                    delay(10);
                }
                value0 = Serial.read();
                while (Serial.available() == 0) {
                    delay(10);
                }
                value1 = Serial.read();
                new_icr1 = (value1 << 8) + value0;
                ICR1 = new_icr1;
            }

        } else if (cmd=='N') {
            // Channel name
            char next_byte;
            char eeprom_idx = 0;

            if (value=='=') {
                while (1) {
                    while (Serial.available() == 0) {
                        delay(10);
                    }
                    next_byte = Serial.read();
                    EEPROM.write( eeprom_idx, next_byte );
                    eeprom_idx++;
                    if (next_byte==0) {
                        break;
                    }
                }
            } else if (value=='?') {
                String out_name="";
                while (1) {
                    next_byte = EEPROM.read( eeprom_idx );
                    if (next_byte==0) {
                        break;
                    }
                    out_name.concat(next_byte);
                    eeprom_idx++;
                }
                send_data_string(&out_name,'N');
            } else {
                // turn LED on as error signal and spin forever
                digitalWrite(LEDPin,HIGH);
                while (1) {
                    delay(100);
                }
            }


        }

    }
}
