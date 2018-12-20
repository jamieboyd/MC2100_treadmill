#include <TimerOne.h>
#include <avr/interrupt.h>


// Constants for input and ouput pins
#define SPEED_UP  10 // PB2 momentary Switch Input to increase speed by 0.1 mph
#define SLOW_DOWN 11 // PB3 momentary switch to decrease speed by 0.1 mph
//#define EM_STOP   12 // PB4 pin for safety device, when it goes low, SHUT DOWN EVERYTHING
#define PWM_OUT   9 // PWM output pin connected to blue wire of MC2100 (50ms period PWM out, 0 to 85% duty cycle)
#define PWM_CYCLE 50.0 //Output Signal PWM Period (50ms)
// Constants for output duty cycle and input speed
#define MAX_DUTY  863 //Max Duty Cycle expected by MC-2100 (85% of 1023)
#define MIN_DUTY  150 //Min Duty Cycle expected by MC-2100 (10% of 1023
#define DUTY_INCR 35 //20 steps from min to max

int PWMduty =0;
// Pin change interrupt for the three input pins buttons and emergency stop
ISR(PCINT0_vect){
  if (digitalRead (SPEED_UP) == LOW){
    if (PWMduty == 0){
      PWMduty = MIN_DUTY;
    }else{
      PWMduty = min (MAX_DUTY, PWMduty + DUTY_INCR);
    }
    Timer1.setPwmDuty(PWM_OUT, PWMduty);
  }else{
    if (digitalRead (SLOW_DOWN) ==LOW){
      if (PWMduty <= MIN_DUTY){
        PWMduty = 0;
      }else{
        PWMduty = max (MIN_DUTY, PWMduty - DUTY_INCR);
      }
      Timer1.setPwmDuty(PWM_OUT, PWMduty);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
   // 2 inputs processed by pin change interrupt
 // pinMode (EM_STOP, INPUT_PULLUP); // Hall sensor for emergency Stop
  pinMode (SLOW_DOWN, INPUT_PULLUP); // button for slowing down
  pinMode (SPEED_UP, INPUT_PULLUP); // button for speeding up
  cli();
  PCMSK0 |= 0b00011100;    // turn on pin PB2,BP3, BP4
  sei();
  // initialize timer 1 for output PWM signal
  TCCR1A = 0;                 // clear control register A
  TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  Timer1.pwm(PWM_OUT, 0, 1000 * PWM_CYCLE);
  Timer1.setPwmDuty(PWM_OUT, MIN_DUTY);
   // initialize the serial communication:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}
