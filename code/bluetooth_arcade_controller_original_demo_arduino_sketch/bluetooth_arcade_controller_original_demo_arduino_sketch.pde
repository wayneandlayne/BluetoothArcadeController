/*
    Wayne and Layne present:
    Bluetooth Arcade controller shield, firmware revision 1.02
    Copyright (c) 2012 Wayne and Layne, LLC
    Updated to work with Arduino 1.0

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    
    For downloads, documentation, and other information:
    http://wwww.wayneandlayne.com/projects/bluetooth-arcade-controller/

    --------------------------------------------------------------------------

    Usage instructions:

    This code currently connects to the WT12 bluetooth module and performs the
    initialization every time it starts up. It will automatically reconnect to
    a previously-paired computer, but it may take a few seconds to reconnect,
    because the host computer has to give up on the old connection before it
    will answer the new connection attempt from the WT12 BT module.

    All the digital pins on the mega are used for active-low (internall pull-
    ups enabled) button inputs, with the exception of these pins:
    0   programming with uart0
    1   programming with uart0
    13  led built into the board, we use it as a special button input
    18  uart1 with WT12 module
    19  uart1 with WT12 module

    We have a mapping from button number to pin number (makes it easy to skip
    the pins we need to skip) and another mapping from button number to the
    actual character code we want to transmit (USB HID record key codes).

    The WT12 bluetooth module is connected to the Arduino Mega's second UART
    port, UART1 on pins 18 and 19. The arduino TX goes to the WT12's RX, and
    vice versa. Since the arduino runs at 5v and the WT12 runs at 3.3v, we
    need to handle the voltage translation for the signal lines. Fortunately,
    there are no bi-directional signal lines, so we just use a resistor
    divider for arduino pins driving WT12 pins, and directly connect WT12 pins
    that are inputs to the arduino.

    We also have an arduino pin (digital 53) connected through a resistor
    divider to the WT12's reset pin, so we can control when the device is reset
    (puts us to a known state on startup). We also have control of the
    command / data mode switch pin (escape), via Arduino pin Analog 4, again,
    through a voltage divider to produce a safe voltage for the WT12.
    Additionally, the WT12 can be set up to output the current carrier detect
    (CD) state to a pin, which we've connected to Arduino pin Analog 5, which
    can be used to easily check if the WT12 is currently connected to a host
    bluetooth device.

    We also planned to add some LEDs to the enclosure for artist effect, and
    set up a simple little system where the leds will "dance" whenever the
    button state changes. We use a silly little array of basically random
    binary values to decide which of the eight LEDs to turn on. We're using
    the upper eight analog pins for these LED outputs. If you want to drive
    something larger than an led, such as a bunch of leds or some EL wire or
    whatever, you're responsible for adding a relay or transistor or whatever
    you need so you have enough drive current.
*/

#include <avr/pgmspace.h>

//------------------------------------------------------------------------------

#define NUM_BUTTONS 52

// This array maps a button index to an arduino pin index,
// skipping pins that need to be skipped (uart pins, pin 13, etc)
byte map_pins[NUM_BUTTONS] = {
     2,  3,  4,  5,  6,  7,  8,  9, 10, 11, //  0 -  9
    12, 14, 15, 16, 17, 20, 21, 22, 23, 24, // 10 - 19
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, // 20 - 29
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44, // 30 - 39
    45, 46, 47, 48, 49, 50, 51, 52, 53, 54, // 40 - 49
    55, 56};                                // 50 - 51 (52 buttons total)

// This array maps a button index to an actual character to transmit
// Check out http://www.usb.org/developers/devclass_docs/Hut1_11.pdf
//   if you want to change the key mappings, starting at page 53.
byte map_char[NUM_BUTTONS] = {
    0x51, 0x50, 0x52, 0x4F, 0x2C, 0x2C, 0x2C, 0x37, 0x37, 0x37, //  0 -  9
    0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, // 10 - 19
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, // 20 - 29
    0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x2F, 0x30, // 30 - 39
    0x31, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x4F, 0x50, 0x51, // 40 - 49
    0x52, 0x28};                                                // 50 - 51 (52 buttons total)

#define WT12_RESET      57
#define WT12_ESCAPE     58
#define WT12_CD         59
#define LED0            60
#define LED1            61
#define BUTTON          13

// we have some pins reserved for off-board LEDs for decoration
#define LED_START       62
#define LED_NUM         8

// we debounce the input buttons to make it awesome
char debounce[NUM_BUTTONS];

//------------------------------------------------------------------------------

prog_char wt12_init_0[] PROGMEM = "SET PROFILE HID ON";
prog_char wt12_init_1[] PROGMEM = "SET BT AUTH * 0000";
prog_char wt12_init_2[] PROGMEM = "SET BT SSP 3 0";
prog_char wt12_init_3[] PROGMEM = "SET BT CLASS 00540";
prog_char wt12_init_4[] PROGMEM = "SET CONTROL CONFIG 0";
prog_char wt12_init_5[] PROGMEM = "SET CONTROL AUTOCALL 0011 500 HID";
prog_char wt12_init_6[] PROGMEM = "SET CONTROL CD 20 0";
prog_char wt12_init_7[] PROGMEM = "SET CONTROL ESCAPE 43 10 1";
prog_char wt12_init_8[] PROGMEM = "SET BT NAME ARCADE";
#define NUM_INIT_MSG 9

PROGMEM const char* string_table[] =
{
    wt12_init_0,
    wt12_init_1,
    wt12_init_2,
    wt12_init_3,
    wt12_init_4,
    wt12_init_5,
    wt12_init_6,
    wt12_init_7,
    wt12_init_8,
};

char buffer[60]; // used for printing these init messages

//------------------------------------------------------------------------------

void setup()
{
    // initialize both serial ports:
    Serial.begin(115200);
    Serial1.begin(115200);

    // on-board input button
    pinMode(BUTTON, INPUT);
    digitalWrite(BUTTON, HIGH); // turn on internal pull-ups

    // initializes the pins used for the regular buttons
    for (byte i = 0; i < NUM_BUTTONS; i++)
    {
        byte pin = map_pins[i];
        pinMode(pin, INPUT); // set pin to input
        digitalWrite(pin, HIGH); // turn on pullup resistor
        debounce[i] == 0; // initialize the debounce array
    }

    // initializes the pins used for the LED outputs
    for (byte i = LED_START; i < (LED_START + LED_NUM); i++)
    {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW); // start off
    }

    init_bluetooth();
}

void init_bluetooth()
{
    // Reset the WT12 module
    pinMode(WT12_RESET, OUTPUT);
    digitalWrite(WT12_RESET, HIGH);
    delay(10);
    digitalWrite(WT12_RESET, LOW);

    Serial.println("\n\nstart");
    Serial1.println("\n\n");
    delay(1000);

    // do the WT12 init
    for (byte i = 0; i < NUM_INIT_MSG; i++)
    {
        strcpy_P(buffer, (char*)pgm_read_word(&(string_table[i])));
        Serial1.println(buffer);
        Serial.println(buffer);
        delay(100);
    }
    delay(1000);
    Serial1.flush(); // it echoes back everything we say?
    Serial1.println("RESET");
}
                                                         
//------------------------------------------------------------------------------

// There are some LEDs connected to the upper eight analog pins (digital 62-69).
// Every time we detect a change in button state, we call do_the_led_dance() which will change the leds.
// Depending on the colors and positions of the 8 leds, we can change the data below to make it look cooler.
byte led_dance[] = {
    // active-high led outputs, only turn on two or three leds at a time
    0b01010100,
    0b10101000,
    0b00101010,
    0b01010100,
    0b00010101,
    0b00101010,
    0b01100100,
    0b01010010,
}; // TODO let's make this into an LFSR, that would be fun
#define NUM_LED_DANCE 8 // make sure this is a power of two! TODO I've seen a nifty preprocessor macro to check if a define is a power of two
#define DANCE_MASK (NUM_LED_DANCE - 1)
byte led_dance_index = 0;

// poke this function to update the leds to the next step in the dance
void do_the_led_dance(void)
{
    led_dance_index = (led_dance_index + 1) & DANCE_MASK;
    byte dance_byte = led_dance[led_dance_index];
    for (byte i = 1; i < LED_NUM; i++)
    {
        if (dance_byte & (1 << ((LED_NUM - 1) - i)))
        {
            digitalWrite(LED_START + i, HIGH);
        }
        else
        {
            digitalWrite(LED_START + i, LOW);
        }
    }
}


//------------------------------------------------------------------------------

/*
	The default Human Interface Device (HID) record specifies that we can
send a maximum of six concurrent keypresses. The way this works is that we only
send a new record whenever the button state changes. That is, we don't send
repeated records if the user holds down a button. We simply report the current
state (what buttons are currently pressed) whenever the button state changes.

	With the next release of the iWRAP firmware used by the WT12 Bluetooth
module, there will be support for defining custom HID records, so we'll be able
to enlarge the record as large as needed to support an effectively unlimited
number of concurrent button presses. We'll let you know when this is released.
*/

#define THRESHOLD 100
#define NUM_KP 6
char kp[NUM_KP];
char kp_prior[NUM_KP];
byte kp_ix = 0;
byte kp_ix_prior;

void send_raw_record(void)
{
    // Look at kp_ix to know how many keypresses we have to report.
    // Data in kp[].
    // An HID raw mode record looks like this:
    // 0x9f 0x0a 0xa1 0x01 0x00 0x00 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
    // Where the last six bytes are the HID key codes.

    Serial1.write((byte)0x9f);
    Serial1.write((byte)0x0a);
    Serial1.write((byte)0xa1);
    Serial1.write((byte)0x01);
    Serial1.write((byte)0x00);
    Serial1.write((byte)0x00);

    char data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for (byte i = 0; i < kp_ix; i++)
    {
        data[i] = kp[i];
    }

    // actually send the record
    Serial1.write(data[0]);
    Serial.print(data[0], HEX);
    Serial1.write(data[1]);
    Serial.print(data[1], HEX);
    Serial1.write(data[2]);
    Serial.print(data[2], HEX);
    Serial1.write(data[3]);
    Serial.print(data[3], HEX);
    Serial1.write(data[4]);
    Serial.print(data[4], HEX);
    Serial1.write(data[5]);
    Serial.println(data[5], HEX);

    do_the_led_dance();
}

void loop(void)
{
    // the actual main loop
    // the bluetooth channel has already been connected
    // we simply send check the buttons' state and send records
    // Supports up to six concurent button presses.
    // Transmit a record whenever a button change occurs. The host assumes that
    // all keys mentioned in a record are held down until the next record is
    // without that key is sent.

    // These two loops will pass any data sent to the Arduino to the WT12,
    // and any data from the WT12 to the computer. This lets you play around
    // with the WT12 from the Arduino serial console to troubleshoot and
    // debug, but you probably want to comment out the second loop (computer to
    // WT12) to avoid accidentally messing things up.
    while (Serial1.available() > 0)
    {
         Serial.write(Serial1.read());
    }
    while (Serial.available() > 0)
    {
         Serial1.write(Serial.read());
    }

    // copy the old keypress state to the proper variable
    memcpy(kp_prior, kp, NUM_KP);
    kp_ix_prior = kp_ix;
    kp_ix = 0;

    for (byte i = 0; i < NUM_BUTTONS; i++)
    {
        byte pin = map_pins[i];
        if (digitalRead(pin) == LOW) // normally pulled-up, buttons connect to ground
        {
            if (debounce[i] < THRESHOLD)
            {
                debounce[i] += 1;
            }
            else
            {
                if (kp_ix < NUM_KP)
                {
                    kp[kp_ix] = map_char[i];
                    kp_ix += 1;
                }
                else
                {
                    // Oh no, we had more than six pressed buttons!
                    Serial.print("full");
                }
            }
        }
        else
        {
            debounce[i] = 0;
        }
    }
    
    // make and transmit an HID raw record, but only if something has changed
    if (kp_ix != kp_ix_prior)
    {
        send_raw_record();
    }
    else
    {
        for (byte i = 0; i < kp_ix; i++)
        {
            if (kp[i] != kp_prior[i])
            {
                send_raw_record();
                break;
            }
        }
    }
}

