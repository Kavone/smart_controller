#include <Arduino.h>
#include <LiquidCrystal.h>

//******************************************************************************
// User adjustable parameters

// LCD pin connections
//        E1_4     E1_3     E1_5     E1_6     E1_7     E1_8
#ifdef __AVR_ATmega2560__
const int RS = 16, EN = 17, D4 = 23, D5 = 25, D6 = 27, D7 = 29;
#else
const int RS = 2, EN = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
#endif

//        E1_1
#ifdef __AVR_ATmega2560__
const int BUZZER = 37;
#else
const int BUZZER = 11;
#endif

// Rotary encoder
// BTN_ENA leads BTN_ENB w/ clockwise rotation
// BTN_ENC is low when button is pressed
//        E1_2           E2_3           E2_5
#ifdef __AVR_ATmega2560__
const int BTN_ENC = 35, BTN_ENA = 31, BTN_ENB = 33;
#else
const int BTN_ENC = 10, BTN_ENA = 8, BTN_ENB = 9;
#endif

// number of "stopping points" in a full revolution of the encoder
const int num_detents_per_revolution = 20;

// number of samples with constant signal value to be considered "debounced"
const int debounce_stable_count = 4;

// time between interrupts for monitoring signal value
// resolution of the timer is 4us so best to make this a multiple of 4
#define ISR_INTERVAL_us 256

// reset push button
#ifdef __AVR_ATmega2560__
const int BTN_RESET = 41;
#endif

// steppers
#ifdef __AVR_ATmega2560__
const int  X_STEP_PIN = 54,  X_DIR_PIN = 55,  X_ENABLE_PIN = 38,  X_CS_PIN = 53;
const int  Y_STEP_PIN = 60,  Y_DIR_PIN =  1,  Y_ENABLE_PIN = 56,  Y_CS_PIN = 49;
const int  Z_STEP_PIN = 46,  Z_DIR_PIN = 48,  Z_ENABLE_PIN = 62,  Z_CS_PIN = 40;
const int E0_STEP_PIN = 26, E0_DIR_PIN = 28, E0_ENABLE_PIN = 24, E0_CS_PIN = 42;
const int E1_STEP_PIN = 36, E1_DIR_PIN = 34, E1_ENABLE_PIN = 30, E1_CS_PIN = 44;
#endif

//******************************************************************************
// calculated parameters which should not need user adjustment

// assume a rising and folling edge of each button happens between detents
const int num_edges_per_revolution = 2 * num_detents_per_revolution;

// Number of timer ticks between interrupts.  Each tick is 4us
#define OCR0A_INCR (ISR_INTERVAL_us / 4)

volatile int pos;
volatile int increments;
volatile int decrements;
volatile long unsigned total_isr_count;
volatile long unsigned total_isr_time;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

/*
 * TIMER0_COMPA_vect
 * ISR triggered by timer0 where inputs are read and debounced.
 * In order to get interrupts at a rate greater than 1 ms (default for timer0),
 * but not require an additional timer and not disturb the timer0 overflow rate
 * (because that is used by default libraries), it makes use of the timer
 * compare interrupt and adjusts the comparison value within the ISR.
 */

ISR(TIMER0_COMPA_vect) {
    static int last_a = 1;
    static int curr_a = 1;
    static int curr_a_stable_count;

    static int initial_b;

    unsigned long isr_start_time;

    isr_start_time = micros();
    total_isr_count++;
    OCR0A = (OCR0A + OCR0A_INCR) & 0xff;

    // get value of B the first time we detect a change on A
    // we assume B is stable by the time A begins to change
    int val = digitalRead(BTN_ENA);
    if ((curr_a_stable_count == 0) && (val != last_a)) {
        initial_b = digitalRead(BTN_ENB);
    }

    // debounce A
    if (val != curr_a) {
        curr_a = val;
        curr_a_stable_count = 1;
    } else {
        if (curr_a_stable_count > 0) {
            curr_a_stable_count++;
        }
    }

    // if values are stable, adjust rotary encoder position
    // B low on rising edge of A indicates clockwise motion while a
    // B high on rising edge of A indicates counter-clockwise motion.
    // B low on falling edge on A indicates counter clockwise motion
    // B high on falling edge of A indicates clockwise motion
    if (curr_a_stable_count == debounce_stable_count) {
        if (last_a != curr_a) {
            if (curr_a == initial_b) {
                pos--;
                if (pos < 0) {
                    pos += num_edges_per_revolution;
                }
            } else {
                pos++;
                if (pos >= num_edges_per_revolution) {
                    pos -= num_edges_per_revolution;
                }
            }

            last_a = curr_a;
        }

        curr_a_stable_count = 0;
    }

    // Now deal with the button press which simply resets the
    // count to zero so we don't care about bounces
    val = digitalRead(BTN_ENC);
    if (val == 0) {
        pos = 0;
        increments = 0;
        decrements = 0;
    }

#ifdef __AVR_ATmega2560__
    val = digitalRead(RESET_SW);
    if (val == 0) {
        pos = 0;
        increments = 0;
        decrements = 0;
    }
#endif

    total_isr_time += (micros() - isr_start_time);
}


void setup() {
    Serial.begin(9600);
    lcd.begin(20, 4);

    // set up stepper motor drivers
#ifdef __AVR_ATmega2560__
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(X_CS_PIN, OUTPUT);
    digitalWrite(X_CS_PIN, LOW);
    digitalWrite(X_ENABLE_PIN, LOW);
    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(X_DIR_PIN, LOW);
#endif

    // Add a pullup to all buttons
    pinMode(BTN_ENA, INPUT_PULLUP);
    pinMode(BTN_ENB, INPUT_PULLUP);
    pinMode(BTN_ENC, INPUT_PULLUP);

#ifdef __AVR_ATmega2560__
    pinMode(RESET_SW, INPUT_PULLUP);
#endif

    // set up a recuring timer interrupt
    cli();                  // disable interrupts

    // Turn off PWM modes of timer0 because such modes prevent the
    // immediate re-assignment of compare values
    TCCR0A &= ~((1 << WGM01) | (1 << WGM00));  // Turn off PWM modes
    TCCR0B &= ~(1 << WGM02);

    OCR0A = 1;              // Set the timer0 compare value.
                            // The ISR will adjust this value to to potentially
                            // produce multiple interrupts during the normal
                            // 0 -> FF count sequence which takes 1024us
    TIMSK0 |= _BV(OCIE0A);  // Enable the timer compare interrupt

    sei();                  // enable interrupts
}

void loop() {
    static unsigned loop_count;
    loop_count++;

#ifdef __AVR_ATmega2560__ 
    static int last_pos = 0;
    if (last_pos < pos) {
        digitalWrite(X_DIR_PIN, LOW);
    }
    else if (last_pos > pos) {
        digitalWrite(X_DIR_PIN, HIGH);
    }

    while (last_pos != pos) {
        for (int i = 0; i < 100; i++) {
            digitalWrite(X_STEP_PIN, HIGH);
            delayMicroseconds(100);
            digitalWrite(X_STEP_PIN, LOW);
            delayMicroseconds(10);
        }
        if (last_pos < pos) {
            last_pos++;
        }
        else {
            last_pos--;
        }
    }
#endif

    lcd.clear();
    lcd.print("Pos: ");
    lcd.print(pos);
    lcd.setCursor(0,1);
    lcd.print("Increments: ");
    lcd.print(increments);
    lcd.setCursor(0,2);
    lcd.print("Decrements: ");
    lcd.print(decrements);

    if (loop_count == 100) {
        loop_count = 0;
        Serial.print("Count: ");
        Serial.print(total_isr_count);
        Serial.print("\n");
        Serial.print("ISR Time: ");
        Serial.print(total_isr_time);
        Serial.print("\n");
        Serial.print("Wall Time: ");
        Serial.print(micros());
        Serial.print("\n\n");
    }
    delay(100);
}
