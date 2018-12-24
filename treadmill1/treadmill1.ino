#include <TimerOne.h>
#include <avr/interrupt.h>
/*
 * Bare bones treadmill control. Goes faster and slower when faster and slower buttons arepressed, 
 * stops when emergency stop Hall sensor is activated.
 */

#define hasSerial 1

// Constants for input and ouput pins
#define SPEED_UP  10 // PB2 momentary Switch Input to increase speed
#define SLOW_DOWN 11 // PB3 momentary switch to decrease speed 
#define EM_STOP   12 // PB4 pin for safety device, when it goes low, SHUT DOWN EVERYTHING
#define PWM_OUT   9 // PWM output pin connected to blue wire of MC2100 (50ms period PWM out, 0 to 85% duty cycle)
#define PWM_CYCLE 50.0 //Output Signal PWM Period (50ms)
// Constants for output duty cycle and input speed
#define MAX_DUTY  863 //Max Duty Cycle expected by MC-2100 (85% of 1023)
#define MIN_DUTY  150 //Min Duty Cycle expected by MC-2100 (15% of 1023
#define DUTY_INCR 35 //20 steps from min to max

volatile int PWMduty = 0;
volatile unsigned long lastSetSpeedInterrupt; // time of last interrupt for Pin Change input

// Pin change interrupt for the faster/slower input pins buttons and emergency stop
ISR(PCINT0_vect){
  if (digitalRead (EM_STOP) ==HIGH){
    PWMduty = 0;
    Timer1.setPwmDuty(PWM_OUT, PWMduty);
#ifdef hasSerial
    Serial.print(PWMduty);
    Serial.print("\nEmergency Stop\n");
#endif
  }else{
    unsigned long thisTime = millis();
     if (thisTime > lastSetSpeedInterrupt + 30){
      lastSetSpeedInterrupt = thisTime;
      if (digitalRead (SPEED_UP) == LOW){
        if (PWMduty == 0){
          PWMduty = MIN_DUTY;
        }else{
          PWMduty = min (MAX_DUTY, PWMduty + DUTY_INCR);
        }
        Timer1.setPwmDuty(PWM_OUT, PWMduty);
#ifdef hasSerial
        Serial.print(PWMduty);
        Serial.print("\nSpeeding Up\n");
#endif
      }else{
        if (digitalRead (SLOW_DOWN) ==LOW){
          if (PWMduty <= MIN_DUTY){
            PWMduty = 0;
          }else{
            PWMduty = max (MIN_DUTY, PWMduty - DUTY_INCR);
          }
          Timer1.setPwmDuty(PWM_OUT, PWMduty);
#ifdef hasSerial 
          Serial.print(PWMduty);
          Serial.print("\nSlowing Down\n");
#endif
        }
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
   // 3 inputs processed by pin change interrupt
  pinMode (SLOW_DOWN, INPUT_PULLUP); // button for slowing down
  pinMode (SPEED_UP, INPUT_PULLUP); // button for speeding up
  pinMode (EM_STOP, INPUT_PULLUP); // Hall sensor for emergency Stop
  cli();
  PCICR |= 0b00000001;    // turn on port B
  PCMSK0 |= 0b00011100;    // turn on pin PB2,BP3, BP4
  sei();
  // initialize timer 1 for output PWM signal
  TCCR1A = 0;                 // clear control register A
  TCCR1B = _BV(WGM13);        // set mode 8: phase and frequency correct pwm, stop the timer
  Timer1.pwm(PWM_OUT, 0, 1000 * PWM_CYCLE);
  Timer1.setPwmDuty(PWM_OUT, MIN_DUTY);
   // initialize the serial communication:
#ifdef hasSerial
  Serial.begin(9600);
  Serial.print ("Set up has set up\n");
#endif
  lastSetSpeedInterrupt =0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
