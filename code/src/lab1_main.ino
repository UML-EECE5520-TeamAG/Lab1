#include <Arduino.h>

//// Processor Constants ///////////////////////////////////////////////

// ATMega2560 has a 16MHz clock
constexpr unsigned long long kClockCycle = 16000000ULL;


//// Pin constants /////////////////////////////////////////////////////

// Switch button is digtial pin 2
constexpr int kButtonPin = 2;

// Buzzer is digital (pwm) pin 3
constexpr int kBuzzerPin = 3;

// Red LED is digital pin 4
constexpr int kRedLedPin = 4;

// Yellow LED is digital pin 5
constexpr int kYellowLedPin = 5;

// Green LED is digital pin 6
constexpr int kGreenLedPin = 6;

// 74HC595 data (DS) is digital pin 7
constexpr int kShiftRegisterDataPin = 7;

// 74HC595 latch (STCP - STorage register Clock Pin) is digital pin 8
constexpr int kShiftRegisterLatchPin = 8;

// 74HC595 clock (SHCP) is digital pin 9
constexpr int kShiftRegisterClockPin = 9;

// Leftmost 7-segment display digit is digital pin 10
constexpr int kDisplayDigitOnePin = 10;

// Center-left 7-segment display digit is digital pin 11
constexpr int kDisplayDigitTwoPin = 11;

// Center-right 7-segment display digit is digital pin 12
constexpr int kDisplayDigitThreePin = 12;

// Rightmost 7-segment display digit is digital pin 13
constexpr int kDisplayDigitFourPin = 13;


//// Hardware Constants ////////////////////////////////////////////////

// The frequency at which the timer is to initiate an interrupt
// (1 Hz = 1 second)
constexpr unsigned int kTimerInterruptFrequency = 1;

// The value at which to increment the timer clock at
// 1 = 16MHz increment
// 8 = 2MHz increment
// 64 = 250kHz increment
// 256 = 62.5kHz increment
// 1024 = 15.625kHz increment
constexpr unsigned int kTimerPrescalar = 1024;

// The value to input into the timer compare match register to get the
// timer to interrupt every interval
constexpr uint16_t kTimerMatchValue = kClockCycle 
        / (kTimerInterruptFrequency * kTimerPrescalar) - 1UL;

// The delay necessary to debounce the hardware switch
constexpr unsigned long kDebounceDelay = 50;

// The value of the timer at which the buzzer triggers
constexpr unsigned int kBuzzerTriggerTime = 3;

// Frequency of buzzer sound in Hz
constexpr unsigned long long kBuzzerFrequency = 4000ULL;

// Period (e.g. cycle of on/off) of the buzzer in milliseconds
constexpr unsigned int kBuzzerPeriod = 250U;

// The delay value between turning the buzzer on and off to acheive the
// desired frequency
constexpr long long kBuzzerTransitionDelay = 1000000LL / (2 * kBuzzerFrequency);

// The period that the red LED flashes on and off at
constexpr int kRedLedFlashPeriod = 300;

// Value indicating digit one on the 4 number 7-segment display
constexpr int kDigitOne = 12;

// Value indicating digit two on the 4 number 7-segment display
constexpr int kDigitTwo = 9;

// Value indicating digit three on the 4 number 7-segment display
constexpr int kDigitThree = 8;

// Value indicating digit four on the 4 number 7-segment display
constexpr int kDigitFour = 6;

// Shift register value indicating that no display segments are on
constexpr int kNoSegmentsOn = 0b11111111;

// Value indicating segment A (top horizontal line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentA = 0b11111110;

// Value indicating segment B (top right vertical line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentB = 0b11111101;

// Value indicating segment C (bottom right vertical line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentC = 0b11111011;

// Value indicating segment D (bottom horizontal line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentD = 0b11110111;

// Value indicating segment E (bottom left vertical line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentE = 0b11101111;

// Value indicating segment F (top left vertical line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentF = 0b11011111;

// Value indicating segment G (middle horizontal line) on one of the
// 7-segment display digits when sent to the shift register
constexpr uint8_t kSegmentG = 0b10111111;

// Value indicating the decimal point on one of the 7-segment display 
// digits when sent to the shift register
constexpr uint8_t kDecimalPoint = 0b01111111;

// Shift register value that will output a 0 digit on the seven segment display
constexpr uint8_t kZero = kSegmentA & kSegmentB & kSegmentC & kSegmentD 
                & kSegmentE & kSegmentF;

// Shift register value that will output a 1 digit on the seven segment display
constexpr uint8_t kOne = kSegmentB & kSegmentC;

// Shift register value that will output a 2 digit on the seven segment display
constexpr uint8_t kTwo = kSegmentA & kSegmentB & kSegmentD & kSegmentE 
                & kSegmentG;

// Shift register value that will output a 3 digit on the seven segment display
constexpr uint8_t kThree = kSegmentA & kSegmentB & kSegmentC & kSegmentD 
                & kSegmentG;

// Shift register value that will output a 4 digit on the seven segment display
constexpr uint8_t kFour = kSegmentB & kSegmentC & kSegmentF & kSegmentG;

// Shift register value that will output a 5 digit on the seven segment display
constexpr uint8_t kFive = kSegmentA& kSegmentC & kSegmentD & kSegmentF 
                & kSegmentG;

// Shift register value that will output a 6 digit on the seven segment display
constexpr uint8_t kSix = kSegmentA & kSegmentC & kSegmentD & kSegmentE 
                & kSegmentF & kSegmentG;

// Shift register value that will output a 7 digit on the seven segment display
constexpr uint8_t kSeven = kSegmentA & kSegmentB & kSegmentC;

// Shift register value that will output a 8 digit on the seven segment display
constexpr uint8_t kEight = kSegmentA & kSegmentB & kSegmentC & kSegmentD 
                & kSegmentE & kSegmentF & kSegmentG;

// Shift register value that will output a 9 digit on the seven segment display
constexpr uint8_t kNine = kSegmentA & kSegmentB & kSegmentC & kSegmentD 
                & kSegmentF & kSegmentG;

// The seven segment digit shift register values for each timer value
// (Zero digit is leftmost digit on the seven segment display that is
// turned on)
const uint8_t timer_digits[100][2] = {
    {kZero,  kZero},    // 00
    {kZero,  kOne},     // 01
    {kZero,  kTwo},     // 02
    {kZero,  kThree},   // 03
    {kZero,  kFour},    // 04
    {kZero,  kFive},    // 05
    {kZero,  kSix},     // 06
    {kZero,  kSeven},   // 07
    {kZero,  kEight},   // 08
    {kZero,  kNine},    // 09
    {kOne,   kZero},    // 10
    {kOne,   kOne},     // 11
    {kOne,   kTwo},     // 12
    {kOne,   kThree},   // 13
    {kOne,   kFour},    // 14
    {kOne,   kFive},    // 15
    {kOne,   kSix},     // 16
    {kOne,   kSeven},   // 17
    {kOne,   kEight},   // 18
    {kOne,   kNine},    // 19
    {kTwo,   kZero},    // 20
    {kTwo,   kOne},     // 21
    {kTwo,   kTwo},     // 22
    {kTwo,   kThree},   // 23
    {kTwo,   kFour},    // 24
    {kTwo,   kFive},    // 25
    {kTwo,   kSix},     // 26
    {kTwo,   kSeven},   // 27
    {kTwo,   kEight},   // 28
    {kTwo,   kNine},    // 29
    {kThree, kZero},    // 30
    {kThree, kOne},     // 31
    {kThree, kTwo},     // 32
    {kThree, kThree},   // 33
    {kThree, kFour},    // 34
    {kThree, kFive},    // 35
    {kThree, kSix},     // 36
    {kThree, kSeven},   // 37
    {kThree, kEight},   // 38
    {kThree, kNine},    // 39
    {kFour,  kZero},    // 40
    {kFour,  kOne},     // 41
    {kFour,  kTwo},     // 42
    {kFour,  kThree},   // 43
    {kFour,  kFour},    // 44
    {kFour,  kFive},    // 45
    {kFour,  kSix},     // 46
    {kFour,  kSeven},   // 47
    {kFour,  kEight},   // 48
    {kFour,  kNine},    // 49
    {kFive,  kZero},    // 50
    {kFive,  kOne},     // 51
    {kFive,  kTwo},     // 52
    {kFive,  kThree},   // 53
    {kFive,  kFour},    // 54
    {kFive,  kFive},    // 55
    {kFive,  kSix},     // 56
    {kFive,  kSeven},   // 57
    {kFive,  kEight},   // 58
    {kFive,  kNine},    // 59
    {kSix,   kZero},    // 60
    {kSix,   kOne},     // 61
    {kSix,   kTwo},     // 62
    {kSix,   kThree},   // 63
    {kSix,   kFour},    // 64
    {kSix,   kFive},    // 65
    {kSix,   kSix},     // 66
    {kSix,   kSeven},   // 67
    {kSix,   kEight},   // 68
    {kSix,   kNine},    // 69
    {kSeven, kZero},    // 70
    {kSeven, kOne},     // 71
    {kSeven, kTwo},     // 72
    {kSeven, kThree},   // 73
    {kSeven, kFour},    // 74
    {kSeven, kFive},    // 75
    {kSeven, kSix},     // 76
    {kSeven, kSeven},   // 77
    {kSeven, kEight},   // 78
    {kSeven, kNine},    // 79
    {kEight, kZero},    // 80
    {kEight, kOne},     // 81
    {kEight, kTwo},     // 82
    {kEight, kThree},   // 83
    {kEight, kFour},    // 84
    {kEight, kFive},    // 85
    {kEight, kSix},     // 86
    {kEight, kSeven},   // 87
    {kEight, kEight},   // 88
    {kEight, kNine},    // 89
    {kNine,  kZero},    // 90
    {kNine,  kOne},     // 91
    {kNine,  kTwo},     // 92
    {kNine,  kThree},   // 93
    {kNine,  kFour},    // 94
    {kNine,  kFive},    // 95
    {kNine,  kSix},     // 96
    {kNine,  kSeven},   // 97
    {kNine,  kEight},   // 98
    {kNine,  kNine}     // 99
};


//// State enums, types, and constants /////////////////////////////////

// An enumeration class representing each possible state of the traffic
// light controller
enum class States : uint8_t {
    kInit,
    kRedLight,
    kGreenLight,
    kYellowLight
};

// The maximum number of states in the state machine
constexpr size_t kMaxStates = 4;

// A defined function pointer type for state functions
typedef void (*StateFunction)();

// A constant array indexed by the state enumerations that contains at
// each index the enumeration value of the next state
const States next_state[kMaxStates] = {
    States::kRedLight,
    States::kGreenLight,
    States::kYellowLight,
    States::kRedLight
};

// A constant array indexed by the state enumerations that contains at
// each index the timer value that triggers the next state transition
const unsigned int state_timer_limit[kMaxStates] {
    0,
    15,
    15,
    3
};

//// Global variables //////////////////////////////////////////////////

// The current FSM state that the arduino is in
volatile States current_state = States::kInit;

// The current value of the timer in seconds
volatile unsigned int timer_secs = 0;

// Boolean indicating whether red LED is active
bool red_led_on = false;

// Time that red led was last toggled
unsigned long red_led_toggle_time = 0;

// Boolean indicating whether buzzer is active
bool buzzer_on = false;

// Time that buzzer was last toggled
unsigned long buzzer_toggle_time = 0;


//// Hardware Control Functions ////////////////////////////////////////

/**
 * @brief Displays the current time on the 4-Digit Seven Segment Display
 * 
 */
void DisplayTime(){

    // Turn off all digits
    digitalWrite(kDisplayDigitOnePin, HIGH);
    digitalWrite(kDisplayDigitTwoPin, HIGH);
    digitalWrite(kDisplayDigitThreePin, HIGH);
    digitalWrite(kDisplayDigitFourPin, HIGH);

    // Turn off shift register output
    digitalWrite(kShiftRegisterLatchPin, LOW);

    // Send tens digit to shift register (need to invert bits for
    // numbers to appear properly)
    shiftOut(kShiftRegisterDataPin, kShiftRegisterClockPin, MSBFIRST, 
        ~timer_digits[timer_secs % 100][0]);

    // Turn on shift register output
    digitalWrite(kShiftRegisterLatchPin, HIGH);

    // Update the tens digit
    digitalWrite(kDisplayDigitThreePin, LOW);

    // Delay briefly
    delay(5);

    // Turn off all digits
    digitalWrite(kDisplayDigitOnePin, HIGH);
    digitalWrite(kDisplayDigitTwoPin, HIGH);
    digitalWrite(kDisplayDigitThreePin, HIGH);
    digitalWrite(kDisplayDigitFourPin, HIGH);

    // Turn off shift register output
    digitalWrite(kShiftRegisterLatchPin, LOW);

    // Send ones digit to shift register (need to invert bits for
    // numbers to appear properly)
    shiftOut(kShiftRegisterDataPin, kShiftRegisterClockPin, MSBFIRST, 
        ~timer_digits[timer_secs % 100][1]);

    // Turn on shift register output
    digitalWrite(kShiftRegisterLatchPin, HIGH);

    // Update the ones digit
    digitalWrite(kDisplayDigitFourPin, LOW);

    // Delay briefly
    delay(5);
}


/**
 * @brief Turns on the green LED light and shuts off all other LEDs
 * 
 */
void GreenLight(){
    digitalWrite(kRedLedPin, LOW);
    digitalWrite(kYellowLedPin, LOW);
    digitalWrite(kGreenLedPin, HIGH);
}

/**
 * @brief Turns on the yellow LED light and shuts off all other LEDs
 * 
 */
void YellowLight(){
    digitalWrite(kRedLedPin, LOW);
    digitalWrite(kYellowLedPin, HIGH);
    digitalWrite(kGreenLedPin, LOW);
}

/**
 * @brief Turns on the red LED light and shuts off all other LEDs
 * 
 */
void RedLight(){
    digitalWrite(kRedLedPin, HIGH);
    digitalWrite(kYellowLedPin, LOW);
    digitalWrite(kGreenLedPin, LOW);
}

/**
 * @brief Flashes the red LED light on and off and keeps all other LEDs
 *        off
 * 
 */
void ToggleRedLight(){
    // Check if the delay time has passed since the last toggle and
    // if so, toggle the red LED again
    if(millis() > red_led_toggle_time + kRedLedFlashPeriod /2){
        // Save current time as the last toggle time
        red_led_toggle_time = millis();

        // Toggle the red LED on variable
        red_led_on = !red_led_on;
        
        // Turn the red LED on or off
        digitalWrite(kYellowLedPin, LOW);
        digitalWrite(kGreenLedPin, LOW);
        digitalWrite(kRedLedPin, red_led_on);
    } 
}

/**
 * @brief Turns on the buzzer for a predetermined amount of time at a
 *        predetermined frequency befor turning it off again
 * 
 */
void BuzzerBeep(){
    // Check to see if the buzzer should be triggered based on the
    // current timer time
    if(timer_secs <= kBuzzerTriggerTime){

        // Check to see if the buxxer should be toggled on/off depending
        // on the buzzer transition delay
        if(millis() > buzzer_toggle_time + kBuzzerTransitionDelay){

            // Save the current time as the buzzer toggle time
            buzzer_toggle_time = millis();
            
            // Toggle the buzzer on variable
            buzzer_on = !buzzer_on;

            // Turn the buzzer on or off
            digitalWrite(kBuzzerPin, buzzer_on);
        }
    }
    else {
        // Buzzer should not be on so ensure that it is off
        digitalWrite(kBuzzerPin, LOW);
    }
    
}

/**
 * @brief Button interrupt function that checks if the interrut was
 *        caused by a button press or noise and if the button was
 *        pressed it will transition state, detach the interrupt, and
 *        enable timer interrupts
 * 
 */
void CheckButton(){
    // A static variable (e.g. state persistent across function calls)
    // that holds the last time that the interrupt function was called
    static unsigned long last_interrupt_time = 0;

    // Time of the current interrupt
    unsigned long current_interrupt_time = millis();

    // Check to see if this interrupt was caused by noise or an actual
    // button press
    if((current_interrupt_time - last_interrupt_time) > kDebounceDelay){
        // Change state
        StateTransition();

        // Remove interrupt attachment (no longer needed)
        detachInterrupt(digitalPinToInterrupt(kButtonPin));

        // Start timer interrupts
        EnableTimerInterrupts();
    }

    // Record current interrupt time for the next interrupt
    last_interrupt_time = current_interrupt_time;
}

//// State Transition Functions ////////////////////////////////////////

/**
 * @brief A function that changes all state-dependent variables upon a
 *        state transition
 * 
 */
void StateTransition(){

    // Set the current state to the next FSM state
    current_state = next_state[static_cast<size_t>(current_state)];

    // Reset the timer to the max countdown time for the state
    timer_secs = state_timer_limit[static_cast<size_t>(current_state)];
}

// Make an array of state function pointers to make it easy
// and fast to transition between each state functionality during the
// loop
const StateFunction state_function[kMaxStates]{
    &ToggleRedLight,
    &RedLight,
    &GreenLight,
    &YellowLight
};


//// Timer Setup and Interrupt Functions ///////////////////////////////

/**
 * @brief Set hardware timer 1 to interrupt at predetermined interval
 * 
 */
void SetupTimer(){

    // Disable interrupts
    cli();

    // Set all of timer/counter control register 1A to 0
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // | COM1A1| COM1A0| COM1B1| COM1B0| COM1C1| COM1C0| WGM11 | WGM10 |
    // *---------------------------------------------------------------*
    // COM1A1:0 - Compare Output Mode for Channel A
    // COM1B1:0 - Compare Output Mode for Channel B
    // COM1C1:0 - Compare Output Mode for Channel C
    // WGM11:0 - Waveform Generation Mode
    // pg. 154 of ATmega2560 datasheet
    TCCR1A = 0;

    // Set all of timer/counter control register 1B to 0
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // | ICNC1 | ICES1 |   -   | WGM13 | WGM12 | CS12  | CS11  | CS10  |
    // *---------------------------------------------------------------*
    // ICNC1 - Input Capture Noise Canceler
    // ICES1 - Input Capture Edge Select
    // WGM13:2 - Waveform Generation Mode
    // CS12:0 - Clock Select
    // pg. 156 of ATmega2560 datasheet
    TCCR1B = 0;

    // Initialize timer counter value  of timer 1 to 0
    TCNT1  = 0;

    // Set output compare match register 1A for predetermined increments
    OCR1A =  kTimerMatchValue;

    // turn on CTC mode
    TCCR1B |= (1 << WGM12);

    // Set CS10 and CS12 bits for 1024 prescaler
    // See pg. 157 of ATmega2560 datasheet
    TCCR1B |= (1 << CS12) | (1 << CS10);  

    // Allow interrupts
    sei();
}

/**
 * @brief Enables the interrupt for timer 1 and starts the counter at 0
 * 
 */
void EnableTimerInterrupts(){
    // Disable interrupts
    cli();

    // Initialize timer counter value  of timer 1 to 0
    TCNT1  = 0;

    // Enable timer compare interrupt through timer interupt mask 
    // register
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // |   -   |   -   | ICIE1 |   -   | OCIE1C| OCIE1B| OCIE1A| TOIE1 |
    // *---------------------------------------------------------------*
    // ICIE1 - Timer/Counter, Input Capture Interupt Enable
    // OCIE1C - Timer/Counter, Output Compare C Match Interrupt Enable
    // OCIE1B - Timer/Counter, Output Compare B Match Interrupt Enable
    // OCIE1A - Timer/Counter, Output Compare A Match Interrupt Enable
    // TOIE1 - Timer/Counter/ Overflow Interrupt Enable
    // pgs. 161 & 162 ATmega2560 datasheet
    TIMSK1 |= (1 << OCIE1A);

    // Allow interrupts
    sei();
}

/**
 * @brief Disables the interrupt for timer 1
 * 
 */
void DisableTimerInterrupts(){
    // Disable interrupts
    cli();

    // Disable timer compare interrupt through timer interupt mask 
    // register
    //     7       6       5       4       3       2       1       0
    // *---------------------------------------------------------------*
    // |   -   |   -   | ICIE1 |   -   | OCIE1C| OCIE1B| OCIE1A| TOIE1 |
    // *---------------------------------------------------------------*
    TIMSK1 &= ~(OCIE1A);

    // Allow interrupts
    sei();
}

/**
 * @brief Construct a timer 1 interrupt handler that increments the
 *        timer variable and, if the timer limit is reached, transitions
 *        the FSM to the next state
 * 
 */
ISR(TIMER1_COMPA_vect){
    
    // Decrement the timer
    timer_secs--;

    // Check to see if timer has reached zero or, if some unforseen
    // event occurs and there is integer overflow, whether the timer
    // value is above the current timer limit
    if(timer_secs == 0 
        || timer_secs > state_timer_limit[static_cast<uint8_t>(current_state)]){
        StateTransition();
    }
}

/**
 * @brief Sets up the arduino microcontroller before it gets into the
 *        main loop
 * 
 */
void setup() {
    // Initialize all variables
    current_state = States::kInit;
    timer_secs = 99;
    red_led_toggle_time = 0;
    buzzer_toggle_time = 0;

    // Set pin modes
    pinMode(kButtonPin, INPUT);
    pinMode(kBuzzerPin, OUTPUT);
    pinMode(kRedLedPin, OUTPUT);
    pinMode(kYellowLedPin, OUTPUT);
    pinMode(kGreenLedPin, OUTPUT);
    pinMode(kShiftRegisterDataPin, OUTPUT);
    pinMode(kShiftRegisterLatchPin, OUTPUT);
    pinMode(kShiftRegisterClockPin, OUTPUT);
    pinMode(kDisplayDigitOnePin, OUTPUT);
    pinMode(kDisplayDigitTwoPin, OUTPUT);
    pinMode(kDisplayDigitThreePin, OUTPUT);
    pinMode(kDisplayDigitFourPin, OUTPUT);

    // Set up timer interrupt for timer 1
    SetupTimer();

    // Set up button interrupt
    attachInterrupt(digitalPinToInterrupt(kButtonPin), CheckButton, RISING);
}

/**
 * @brief Arduino microcontroller main loop
 * 
 */
void loop() {
    state_function[static_cast<size_t>(current_state)]();
    BuzzerBeep();
    DisplayTime();
}