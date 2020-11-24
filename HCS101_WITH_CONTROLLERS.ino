/*
░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░   
░░░░░░░░▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░   - The purpose of this program is - to decode the HCS101 encoder for uses in either RF or IR transmission
░░░░░░░░░░░░▒░░░░░░░░░░░░░░░░░░░░░   - To drive a DRV8837CDSGR motor controller and
░▒▒▒▒▒░░▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░   - To toggle a laser
░▒░░░▒░░▒░░░░░░░░░░░░░░░░░░░░░░░░░
░▒▒▒▒▒▒▒▒░░░░▒▒▒▒▒▒░▒▒▒▒▒▒▒░▒░░░░░
░░░░░▒░░░░░░░▒░░░░░░░░░▒░░░░▒░░░░░
░▒▒▒▒▒░░░░░░░▒░░░░░░░░░▒░░░░▒░░░░░
░▒░░░░░░░░░░░▒▒▒▒░░░░░░▒░░░░▒░░░░░
░▒▒▒▒▒░░▒░░░░▒░░░░░░░░░▒░░░░▒░░░░░
░░░░░▒░░▒░░░░▒░░░░░░░░░▒░░░░▒░░░░░
░░░░░▒▒▒▒░░░░▒░░░░░░░░░▒░░░░▒▒▒▒▒░
░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
**********************************************************
*** PROPERTY OF FUTURE TECHNOLOGIIES LABORATORIES, LLC ***
**********************************************************
*/
                                                                                                      
#include "HCS101DATA_BLOCK.h"

#define PREAMBLE_LED   13               // Datapin for Rx verification - drives onboard LED - compatible with uno and nano
#define DATA_LED       13               // Datapin for Rx verification - drives onboard LED - compatible with uno and nano
#define MAX_BITS       64               // Max transmission msg bits according to hcs101 datasheet (chance code word could be 64 bits needs research)
#define LAZER          7                // Laser pin
#define MAX_T          100              // Max tolerance when measuring Te. Compatible with HCS101 and HCS200
#define MOTOR_SLP_PIN  10               // Pin to toggle motor sleep state DRV8837CDSGR, PIN 10 is PB2
#define MOTOR_FORWARD  PD5              // IN1 for motor controller 
#define MOTOR_BACKWARD PD6              // IN2 for motor controller
#define ON             200              // Analog value to write to motor (0,255)
#define OFF            LOW              // Digital value to write to motorcontrol
#define MOTOR_TIMEOUT  100              // Time in millis that can elapse before motors turn off after button press not recieved

enum RxState {
    RS_NOSYNC =    0,                   // Receiver is inactive, either message was correctly recieved, or waiting for preamble.
    RS_PREAMBLE =  1,                   // Oscillating high and low pulses every Te
    RS_DATA =      2,                   // State in which the arduino is ready to print to the buffer
    RS_COMPLETED = 3                    // Flags the main loop to read/print the buffer.
};

enum RbType {                           // Flags for the pulsewidth validity
    RB_SHORT = 0,                       
    RB_LONG =  1,                       
    RB_ERROR = 2                        
};

unsigned long motorOnTimer =     0;     // Running timer the ticks every time motor is set to on
unsigned long last_timestamp =   0;     // On interrupt change, set this value to the previous time value to make a measurement
unsigned      last_pulse_width = 0;     // Used to determine the message state of the signal

char          rx_bit_count = 0;         // Number of bits already recieved
volatile char rx_state = RS_NOSYNC;     // Initialize state 
uint32_t      rx_buf[3];                // 3x32 long buffer to contain bits recieved
unsigned      tx_clock = 0;             // tx_clock will set the Te shown in the hcs101 datasheet, this value will be dynamic for each message although typcially 420us.

bool          isLaserOn = false;
bool          isMotorForward = false;
bool          isMotorBackward = false;  

void setup()
{
  initialize();
}

void loop()
{
    digitalWrite (DATA_LED,     rx_state == RS_DATA);
    digitalWrite (PREAMBLE_LED, rx_state == RS_PREAMBLE);

    if (rx_state == RS_COMPLETED) {
        if (rx_bit_count >= MAX_BITS) {
            HCS101_keycode keycode;
            HCS101Decode(rx_buf, &keycode);                            // Decodes by places values against a bitmask and containers
            buttonPress(&keycode);                                     // Check which button was pressed
            //HCS101Print(&keycode);                                   // If buffer is complete print it - uncomment line and function for debug                               
        }
        rx_state = RS_NOSYNC;                                          // Reset the interrupt to begin recieve 
    }
    isMotorTimedOut();                                                       // Checks for motor timeout so they don't stay on. 
}

void pin2ISR()                                                         // Interrupt handler, on change
{
  unsigned long timestamp   = micros();                                // Set current time
  unsigned long pulse_width = timestamp - last_timestamp;              // Set pulse width 
  int pin                   = digitalRead(2);                          // get HIGH or LOW from interrupt pin

  switch (rx_state) {
  case RS_NOSYNC:                                                      // If singal pulse is |¯|_____________________  High pulse followed by long low pulse            
      if (pin == 1 && pulse_width > MAX_T*100 && pulse_width < MAX_T*500) {
          rx_state = RS_PREAMBLE;                                      // If sync pulse recieved set the RS state to the next state
          tx_clock = last_pulse_width;                                 // Grab the Te from the first signal pulse " |¯| "
      }
      break;
  case RS_PREAMBLE:                                                    // If singal pulse is |¯|_|¯|_|¯|_|¯|_|¯|_|¯|_  High and Low pulse oscillating every Te
      if (pulse_width < 2 * tx_clock) {
          tx_clock = (tx_clock + pulse_width) >> 1;                    // 
      } else if (pin == 1 && pulse_width > MAX_T*10) {                     // 
          rx_state = RS_DATA;                                          // If preamble was valid, set RS to the next state
          rx_bit_count = 0;                                            // Reset the bit count for the data buffer
          memset(rx_buf, 0, sizeof(rx_buf));                           
      } else {
          rx_state = RS_NOSYNC;                                        // Transmission error
      }
      break;
  case RS_DATA:                                                        // |¯¯|_ for bit 1, |¯|__ for 0, for a total of 66 bit message
      if (pin == 1) {
          int first =  Classify(last_pulse_width);
          int second = Classify(pulse_width);
          if (first == RB_LONG && second == RB_SHORT) {                // Received a 0 bit |¯¯|_
              int idx = rx_bit_count/32;                             
              rx_buf[idx] >>= 1;
              rx_buf[idx] |=  0x80000000;
              rx_bit_count++;
          }
          else if (first == RB_SHORT && second == RB_LONG) {           // Received a 1 bit |¯|__
              int idx = rx_bit_count/32;
              rx_buf[idx] >>= 1;
              rx_bit_count++;
          }
          else {                                                       // Invalid pulse combination, reset to a NOSYNC signal
              rx_state = RS_NOSYNC;
          }
      }
      if (rx_bit_count >= MAX_BITS) {
          rx_state = RS_COMPLETED;
      }
      break;
  }

  last_timestamp = timestamp;                                          // Set the last timestamp to the current for the next interrupt
  last_pulse_width = pulse_width;                                      // Set the last pulse width to the current for the next interrupt
}

int Classify(unsigned pulse)
{
    int d = pulse - tx_clock;                                          // Get (pulsewidth - Te) to determine if it's within an acceptable range
    if (d < -MAX_T) {                                                    
        return RB_ERROR;                                               // If the recieved bit (RB) is not within range return an error
    }
    else if (d < MAX_T) {
        return RB_SHORT;
    }
    else {
        d -= tx_clock;                                                
        if (d < -MAX_T) {
            return RB_ERROR;
        }
        else if (d < MAX_T) {
            return RB_LONG;
        }
        else {
            return RB_ERROR;
        }
    }
}
void HCS101Decode(uint32_t *rx_buf, HCS101_keycode *out)
{
    out->serial = rx_buf[1];
    out->buttons = (~(rx_buf[1] >> 28)) & 0xF; //Buttons are sent negated
}

void buttonPress(const HCS101_keycode *keycode)
{
    if (keycode->buttons & BM_S0) {
      toggleLaser();
    }
    if (keycode->buttons & BM_S1) {
      motorForward();
    }
    if (keycode->buttons & BM_S2) {
      motorBackward();
    } 
}

/* //OPTIONAL FOR DEBUG ONLY --- UNTOGGLE IN MAIN LOOP ALSO
void HCS101Print(const HCS101_keycode *keycode)
{
    Serial.print("ID# ");
    Serial.print(keycode->serial, HEX);
    Serial.print(", Button#");
    if (keycode->buttons & BM_S0) {
        Serial.print(" 1 ");
    }
    if (keycode->buttons & BM_S1) {
        Serial.print(" 2 ");
    }
    if (keycode->buttons & BM_S2) {
        Serial.print(" 3 ");
    }
    
    Serial.print("\n");
}
*/

void initialize()                            
{
  Serial.begin(38400);                // Could be lower, needs testing                                             
  
  pinMode     (2,              INPUT);
  pinMode     (PREAMBLE_LED,   OUTPUT);
  pinMode     (DATA_LED,       OUTPUT);
  pinMode     (MOTOR_FORWARD,  OUTPUT);
  pinMode     (MOTOR_BACKWARD, OUTPUT);
  pinMode     (MOTOR_SLP_PIN,  OUTPUT);
  
  /*LASER PIN IS PIN 7 AND SET IN 
  turnLaserOn() AND turn LaserOff() FOR SINK*/
  
  attachInterrupt (digitalPinToInterrupt(2), pin2ISR, CHANGE);
  Serial.println  ("Init Success");
}

void toggleLaser()
{
  isLaserOn = !isLaserOn;             // Everytime the laser button is pressed, toggle the laser state
  if(isLaserOn) {
    turnLaserOn();
  }
  else {
    turnLaserOff();
  }
}

void motorForward()
{
  motorOnTimer = millis();
  digitalWrite(MOTOR_SLP_PIN, HIGH);
  analogWrite (MOTOR_BACKWARD, OFF);
  analogWrite (MOTOR_FORWARD, HIGH);
  
}

void motorBackward()
{
  motorOnTimer = millis();
  digitalWrite(MOTOR_SLP_PIN, HIGH);
  analogWrite (MOTOR_FORWARD, OFF);
  analogWrite (MOTOR_BACKWARD, HIGH);
}

void turnLaserOn()
{
 pinMode(LAZER, OUTPUT);
 digitalWrite(LAZER,LOW);
}

void turnLaserOff()
{
 pinMode(LAZER, INPUT);
 digitalWrite(LAZER, HIGH);
}

void isMotorTimedOut()                             
{
   /*if motor has been past timeout period
    * turn the motor off */
    
  if(millis()-motorOnTimer > MOTOR_TIMEOUT){
    digitalWrite(MOTOR_SLP_PIN, LOW);         
  }
}
