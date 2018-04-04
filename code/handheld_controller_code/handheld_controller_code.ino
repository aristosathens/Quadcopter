/* Handheld Controller Code
 * Aristos Athens
 * Began March 2018
 * 
 * Arduino Nano Connections
 *  Joystick:
 * 
 *  Buttons:
 *    Button1 - D8
 *    Button2 - D9
 *    Button3 - D10
 *    Button4 - D11
 *  Wireless Transmitter:
 *    VCC - 5V
 *    GND - GND
 *    Data - D12
 *  LED:
 *    LED - D13
*/

/* -----------------------------------------------------------------------------------------------------------------------
 * Includes, Variables, Defines, Prototypes
 * -----------------------------------------------------------------------------------------------------------------------*/

//Includes
#include <VirtualWire.h>                 //This is a library that handles details of wireless RF communication.
#include <stdlib.h>                      //Use this for string/int conversion functions

//Defines
#define LED_ON PORTB |= B00100000;                      //Turn LED on.
#define LED_OFF PORTB |= B00000000;                     //Turn LED off.

#define TRANSMITTER_PIN 12

#define READ_X_ANALOG PINC & B00000001                  //A0
#define READ_Y_ANALOG PINC & B00000010                  //A1
#define READ_Z_ANALOG PINC & B00000100                  //A2

#define READ_BUTTON_1 PINB & B00000001                  //D8
#define READ_BUTTON_2 PINB & B00000010                  //D9
#define READ_BUTTON_3 PINB & B00000100                  //D10
#define READ_BUTTON_4 PINB & B00001000                  //D11
#define READ_BUTTON_5 PIND & B01000000                  //D6
#define READ_BUTTON_6 PIND & B10000000                  //D7

//Global Variables
long loop_timer;

/* -----------------------------------------------------------------------------------------------------------------------
 * setup() and loop()
 * -----------------------------------------------------------------------------------------------------------------------*/

void setup() {
  Serial.begin(9600);                    // Debugging only
  Serial.println("Beginning setup...");
  init_transmitter();
  init_pins();
  Serial.println("Exiting setup...");
}

void loop() {
  static uint8_t data[4];
  read_data(data);
  transmit_data(data);
  wait();
}

/* -----------------------------------------------------------------------------------------------------------------------
 * Init Pins
 * -----------------------------------------------------------------------------------------------------------------------*/

void init_pins(){
  //Pins default as inputs. No action necessary for those
  DDRB |= B00110000;                                                    //Set pin D12 (LED) and D13(transmitter) as outputs
}

/* -----------------------------------------------------------------------------------------------------------------------
 * Transmitter
 * -----------------------------------------------------------------------------------------------------------------------*/

void init_transmitter(){
  vw_set_tx_pin(TRANSMITTER_PIN);
  vw_set_ptt_inverted(true);            // Required for DR3100
  vw_setup(2000);                       // Bits per sec
}

void read_data(uint8_t data[]){
  data[0] = (uint8_t) 63; //READ_X_ANALOG;
  data[1] = (uint8_t) 21; //READ_Y_ANALOG;
  data[2] = (uint8_t) 74; //READ_Z_ANALOG;
  data[3] = (uint8_t) 29; //(READ_BUTTON_1 | READ_BUTTON_2 << 1 | READ_BUTTON_3 << 2 | READ_BUTTON_4 << 3);
  Serial.print("data[0] is: ");
  Serial.println((int)data[0]);
  Serial.print("data[1] is: ");
  Serial.println((int)data[1]);
  Serial.print("data[2] is: ");
  Serial.println((int)data[2]);
  Serial.print("data[3] is: ");
  Serial.println((int)data[3]);
}

void transmit_data(uint8_t data[]){
  LED_ON;
  vw_send((byte *)data, (uint8_t) 2*sizeof(data));
  vw_wait_tx();                         // Wait for message to finish
  LED_OFF;
}

void wait(){
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}

