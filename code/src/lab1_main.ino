#include <Arduino.h>

//// Processor Constants ///////////////////////////////////////////////

// ATMega2560 has a 16MHz clock
constexpr unsigned long long kClockCycle = 16000000ULL;


//// Pin constants /////////////////////////////////////////////////////

// Switch button is digtial pin 3
constexpr int kButtonPin = 3;

// Buzzer is digital (pwm) pin 12
constexpr int kBuzzerPin = 12;

// Red LED is digital pin 5
constexpr int kRedLedPin = 5;

// Yellow LED is digital pin 6
constexpr int kYellowLedPin = 6;

// Green LED is digital pin 7
constexpr int kGreenLedPin = 7;

// 74HC595 data (DS) is digital pin 8
constexpr int kShiftRegisterDataPin = 8;

// 74HC595 latch (STCP) is digital pin 9
constexpr int kShiftRegisterLatchPin = 9;

// 74HC595 clock (SHCP) is digital pin 10
constexpr int kShiftRegisterClockPin = 10;


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


// Frequency of buzzer sound in Hz
constexpr unsigned long long kBuzzerFrequency = 4000ULL;

// Period (e.g. cycle of on/off) of the buzzer in milliseconds
constexpr unsigned int kBuzzerPeriod = 250U;

// The delay value between turning the buzzer on and off to acheive the
// desired frequency
constexpr long long kBuzzerTransitionDelay = 1000000LL / (2 * kBuzzerFrequency);

// The period that the red LED flashes on and off at
constexpr int kRedLedFlashPeriod = 300;


//// State enums, types, and constants /////////////////////////////////

// An enumeration class representing each possible state of the traffic
// light controller
enum class States : uint8_t {
    kInit,
    kRedLight,
    kRedLightIminentChange,
    kGreenLight,
    kGreenLightIminentChange,
    kYellowLight
};

// The maximum number of states in the state machine
constexpr size_t kMaxStates = 6;

// A defined function pointer type for state functions
typedef void (*StateFunction)();

// A constant array indexed by the state enumerations that contains at
// each index the enumeration value of the next state
const States next_state[kMaxStates] = {
    States::kRedLight,
    States::kRedLightIminentChange,
    States::kGreenLight,
    States::kGreenLightIminentChange,
    States::kYellowLight,
    States::kRedLight
};

// A constant array indexed by the state enumerations that contains at
// each index the timer value that triggers the next state transition
const unsigned int state_timer_limit[kMaxStates] {
    0,
    12,
    15,
    12,
    15,
    3
};

// A constant array indexed by the state enumerations that contains at
// each index the value that the timer should be set to upon transition
// to the state used as the array index
const unsigned int state_timer_reset[kMaxStates] {
    0,
    0,
    12,
    0,
    12,
    0
};


//// Global variables //////////////////////////////////////////////////

// The current FSM state that the arduino is in
volatile States current_state = States::kInit;

// The current value of the timer in seconds
volatile unsigned int timer_secs = 0;

// The timer value that triggers the next state transition
unsigned int timer_limit = 0;


//// Hardware Control Functions ////////////////////////////////////////

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
void FlashRedLight(){
    digitalWrite(kYellowLedPin, LOW);
    digitalWrite(kGreenLedPin, LOW);

    digitalWrite(kRedLedPin, HIGH);
    delay(kRedLedFlashPeriod / 2);
    digitalWrite(kRedLedPin, LOW);
    delay(kRedLedFlashPeriod / 2);
}

/**
 * @brief Turns on the buzzer for a predetermined amount of time at a
 *        predetermined frequency befor turning it off again
 * 
 */
void BuzzerBeep(){
    digitalWrite(kBuzzerPin, HIGH);
    delay(kBuzzerTransitionDelay);
    digitalWrite(kBuzzerPin, LOW);
    delay(kBuzzerTransitionDelay);
}


//// State Transition Functions ////////////////////////////////////////

/**
 * @brief A function that changes all state-dependent variables upon a
 *        state transition
 * 
 */
void StateTransition(){
    current_state = next_state[static_cast<size_t>(current_state)];
    timer_secs = state_timer_reset[static_cast<size_t>(current_state)];
    timer_limit = state_timer_limit[static_cast<size_t>(current_state)];
}

/**
 * @brief An interrupt function that changes state from the intial state
 *        upon a button press (triggered on the release of the button)
 *        and disables the interrupt from that point forward
 * 
 */
void InitTransition(){
    // Change state
    StateTransition();

    // Remove interrupt attachment (no longer needed)
    detachInterrupt(digitalPinToInterrupt(kButtonPin));

    // Start timer interrupts
    EnableTimerInterrupts();
}


//// State Functions ///////////////////////////////////////////////////

/**
 * @brief A function to perform operations intrinsic to the initial
 *        FSM state
 * 
 */
void InitState(){
    FlashRedLight();
}

/**
 * @brief A function to perform operations intrinsic to the red light
 *        state (before the timer reaches 3 seconds)
 * 
 */
void RedLightState(){
    RedLight();
}

/**
 * @brief A function to perform operations intrinsic to the red light
 *        state as its about to change (the timer is 3 seconds or less)
 * 
 */
void RedLightIminentChangeState(){
    RedLight();
    BuzzerBeep();
}

/**
 * @brief A function to perform operations intrinsic to the green light
 *        state (before the timer reaches 3 seconds)
 * 
 */
void GreenLightState(){
    GreenLight();
}

/**
 * @brief A function to perform operations intrinsic to the green light
 *        state as its about to change (the timer is 3 seconds or less)
 * 
 */
void GreenLightIminentChangeState(){
    GreenLight();
    BuzzerBeep();
}

/**
 * @brief A function to perform operations intrinsic to the yellow light
 *        state (before the timer reaches 3 seconds)
 * 
 */
void YellowLightState(){
    YellowLight();
    BuzzerBeep();
}

// Make an array of state function pointers to make it easy
// and fast to transition between each state during the loop
const StateFunction state_function[kMaxStates]{
    &InitState,
    &RedLightState,
    &RedLightIminentChangeState,
    &GreenLightState,
    &GreenLightIminentChangeState,
    &YellowLightState
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
    timer_secs++;
    if(timer_secs >= timer_limit){
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
    timer_secs = 0;
    timer_limit = 0;

    // Set pin modes
    pinMode(kButtonPin, INPUT);
    pinMode(kBuzzerPin, OUTPUT);
    pinMode(kRedLedPin, OUTPUT);
    pinMode(kYellowLedPin, OUTPUT);
    pinMode(kGreenLedPin, OUTPUT);
    pinMode(kShiftRegisterDataPin, OUTPUT);
    pinMode(kShiftRegisterLatchPin, OUTPUT);
    pinMode(kShiftRegisterClockPin, OUTPUT);

    // Set up timer interrupt for timer 1
    SetupTimer();

    // Set up button interrupt
    attachInterrupt(digitalPinToInterrupt(kButtonPin), InitTransition, FALLING);
}

/**
 * @brief Arduino microcontroller main loop
 * 
 */
void loop() {
    state_function[static_cast<size_t>(current_state)]();
}