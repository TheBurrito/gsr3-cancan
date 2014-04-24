#ifndef PINS_H_
#define PINS_H_

#define m2EncA 3               // pin 3 (PWM / INT1)
    // Motor Controller D2     // pin 4
#define m2EncB 5               // pin 6 (PWM)
#define m1EncA 6               // pin 5 (PWM)
    // Motor Controller M1DIR  // pin 7 
    // Motor Controller M2DIR  // pin 8 (PCINT0)
    // Motor Controller M1PWM  // pin 9 (PWM / PCINT1 / OC1A)
    // Motor Controller M2PWM  // pin 10 (PWM / SS / PCINT2 / OC1B)
#define m1EncB 11              // pin 11 (PWM / MOSI / PCINT3 / OC2A)
    // Motor Controller SF     // pin 12 (MISO / PCINT4)
#define LED 13                 // pin 13 (SCK / PCINT5)
    // Motor Controller M1FB   // pin A0 (14)
    // Motor Controller M2FB   // pin A1 (15)
#define SNR_PW 16  // Sonar    // pin A2 (16)
#define SNR_RX 17  // Sonar    // pin A3 (17) 
    // pin A4 (18) (I2C SDA)
    // pin A5 (19) (I2C SCL)
#define SERVO_G_FB A6          // pin A6 (20)  (Servo feedback)
// pin A7 (21)
// pin A8 (22)
// pin A9 (23)
#define IR_R   A12             // pin A12 (Analog Only)
#define IRB_FR 24 // Bumper    // pin 24  
#define IRB_F  25 // Bumper    // pin 25 (PWM)
#define IR_FR A15              // pin 26 (A15)
#define IR_F  A16              // pin 27 (A16)
// pin 28 (A17)
#define IRB_FL 29 // Bumper    // pin 29 (A18)
// pin 30 (A19)
#define IR_FL  A20             // pin 31 (A20)
extern int SERVO_G;            // pin 32 (PWM)
// pin 33
#define IR_L   A13             // pin A13 (Analog Only)

#endif //PINS_H_
