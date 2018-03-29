#include "VirtualWire.h"
#include "DHT.h"
#include "string.h"

#define DHTPIN 5     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)

const int led_pin = 11;
const int transmit_pin = 12;
//const int receive_pin = 2;
//const int transmit_en_pin = 3;
DHT dht(DHTPIN, DHTTYPE);
void setup()
{
  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
 // vw_set_rx_pin(receive_pin);
//  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);	 // Bits per sec
  dht.begin();
   Serial.begin(9600);	// Debugging only
   Serial.println("DHT22 Tx");
}



char buffT[6];
char buffH[6];
char buff[15];

void loop()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float tf = t*9.0/5.0 + 32.0;
  
  // check if returns are valid, if they are NaN (not a number) then something went wrong!
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT");
  } else {
    Serial.print("H="); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("T= "); 
    Serial.print(tf);
    Serial.println(" *F");
    dtostrf(tf, 5,2, buffT);
    Serial.print("buffT= "); 
    Serial.println(buffT);
    dtostrf(h, 5,2, buffH);
    Serial.print("buffH= "); 
    Serial.println(buffH);
    buff[0]='\0';
    strcat(buff, buffT);
    strcat(buff, " ");
    strcat(buff, buffH);
      

    Serial.print("Mess= "); 
    Serial.println(buff);
    digitalWrite(led_pin, HIGH); // Flash a light to show transmitting
    vw_send((byte *)buff, strlen(buff));
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(led_pin, LOW);
    delay(10000);
  }
  


}
