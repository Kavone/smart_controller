#include <Arduino.h>
#include <LiquidCrystal.h>

//        E1_4     E1_3     E1_5     E1_6     E1_7     E1_8
#ifdef __AVR_ATmega2560__
const int rs = 16, en = 17, d4 = 23, d5 = 25, d6 = 27, d7 = 29;
#else
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
#endif
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//        E1_1
#ifdef __AVR_ATmega2560__
const int buzzer = 37;
#else
const int buzzer = 11;
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

// E2_8
#ifdef __AVR_ATmega2560__
const int RESET_SW = 41;
#endif

// steppers
#ifdef __AVR_ATmega2560__
const int  X_STEP_PIN = 54,  X_DIR_PIN = 55,  X_ENABLE_PIN = 38,  X_CS_PIN = 53;
const int  Y_STEP_PIN = 60,  Y_DIR_PIN =  1,  Y_ENABLE_PIN = 56,  Y_CS_PIN = 49;
const int  Z_STEP_PIN = 46,  Z_DIR_PIN = 48,  Z_ENABLE_PIN = 62,  Z_CS_PIN = 40;
const int E0_STEP_PIN = 26, E0_DIR_PIN = 28, E0_ENABLE_PIN = 24, E0_CS_PIN = 42;
const int E1_STEP_PIN = 36, E1_DIR_PIN = 34, E1_ENABLE_PIN = 30, E1_CS_PIN = 44;
#endif

const int button_check_interval = 1;
const int debounce_stable_count = 5;
volatile int pos;
volatile int increments;
volatile int decrements;

/*
 * TIMER0_COMPA_vect and TIMER0_COMPB_vect
 * ISR triggered by timer0 where inputs are read and debounced.
 */
#ifdef DUAL_INTERRUPTS
ISR(TIMER0_COMPB_vect, ISR_ALIASOF(TIMER0_COMPA_vect));
#endif
ISR(TIMER0_COMPA_vect) {
    static int isr_count;

    static int last_a = 1;
    static int curr_a = 1;
    static int curr_a_stable_count;

    static int initial_b;

    isr_count++;

    if (isr_count == button_check_interval) {
        isr_count = 0;

        // First deal with the rotary encoder
        // This is done by checking the value of button B when
        // a rising edge on button A is found.  A rising edge on
        // A when B is low indicates clockwise motion while a rising
        // edge on A when B is high indicates counter-clockwise motion

        // get value of B if we are just now starting to debounce A
        int val = digitalRead(BTN_ENA);
        if ((curr_a_stable_count == 0) && (val == 1) && (last_a == 0)) {
            initial_b = digitalRead(BTN_ENB);
        }

        // debounce A
        if (val != curr_a) {
            curr_a = val;
            curr_a_stable_count = 1;
        }
        else {
            if ((curr_a_stable_count > 0)
                  && (curr_a_stable_count < debounce_stable_count)) {
                curr_a_stable_count++;
            }
            else {
                curr_a_stable_count = 0;
            }
        }

        // if values are stable and rising edge on A, update position
        if (curr_a_stable_count == 0)  {
            if ((last_a == 0) && (curr_a == 1)) {
                if (initial_b == 0) {
                    increments++;
                    pos++;
                }
                else {
                    decrements++;
                    pos--;
                }
                if (pos < 0) {
                    pos += 20;
                }
                else if (pos > 19) {
                    pos -= 20;
                }
            }

            last_a = curr_a;
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
    }
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

    // set up a recuring timer interrupt at 1ms
    cli();                  // disable interrupts
    OCR0A = 0x81;           // set timer0/A to generate output when it hits 0x80
                            // pretty much anything is fine here as it will simply
                            // cause output once every time through the counter
                            // 0 -> FF sequence which takes 1024us
    TIMSK0 |= _BV(OCIE0A);  // Set the compare A output to cause an interrupt
#ifdef DUAL_INTERRUPTS
    OCR0B = 0x01;           // set timer0/B to generate output when it hits 0x01
    TIMSK0 |= _BV(OCIE0B);  // Set the compare B output to cause an interrupt
#endif
    sei();                  // enable interrupts
}

void loop() {
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
    delay(10);
}
