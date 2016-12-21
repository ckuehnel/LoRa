/*
 * LoRa Node w/ radino32 SX1272
 * based on Trasnmitter example by In-Circuit.de
 * for more information: www.in-circuit.de or www.radino.cc
 */
 
#include <SPI.h>
#include <stm32/l1/iwdg.h>
#include <radino32_sx1272.h>

// Define led pin
const int PIN_LED =  13;
const int sensorPin = 0;

// Payloadsize for transmission with LoRa
#define TX_PAYLOAD_SIZE                             12
// Define buffer with max buffersize for LoRa
#define BUFFER_SIZE                                 RF_BUFFER_SIZE_MAX 
static char Buffer[BUFFER_SIZE];	    // RF buffer

tRadioDriver *Radio = NULL;

void getTemperature()
{
  int reading = analogRead(sensorPin);
  float voltage = reading * 5.0 / 1024;
  float temperatureC = (voltage - 0.5) * 100;
  dtostrf(temperatureC, 5, 1, Buffer);
  strcat(Buffer, " *C\n");
  Serial.print("Measured temperature = ");
  Serial.print(temperatureC);
  Serial.println(" *C");
};


void setup ( void )
{
    // Init led
    pinMode(PIN_LED, OUTPUT);

    // Set led on
    digitalWrite(PIN_LED, HIGH);
    
    // Start Serial communication
    Serial.begin(115200);
    delay(4000);
    Serial.print("Setup LoRa Node");
    
    // Start init for RF-chip
    BoardInit( );
    Serial.print("-");
    delay(1);
    
    Radio = RadioDriverInit( );
    Serial.print("-");
    delay(1);

    Radio->Init( );
    Serial.print("-");
    delay(1);

    Radio->StartRx( );
    Serial.print("-");
    delay(1);

    delay(500);
    // Set led off
    digitalWrite(PIN_LED, LOW);

    Serial.println(" pass");
}


void loop( void )
{
    int i;
    
    // Activate full power transmission
    SX1272LoRaSetPa20dBm(true);

    // function for running LoRa
    switch( Radio->Process( ) )
    {
    case RF_RX_TIMEOUT:
    case RF_RX_DONE:
    case RF_TX_DONE:
        // Set led off to show end of transmission
        digitalWrite(PIN_LED, LOW);
        
        // Output for finished transmission
        Serial.println("TX done");
        
        if(TX_PAYLOAD_SIZE <= BUFFER_SIZE)
        {
            getTemperature();
            Serial.print("LoRa Message: ");
            Serial.write(Buffer, 8);
            Serial.println();
        }
        
        // Delay between two transmissions
        delay(15000);

        // Start transmission with LoRa
        Radio->SetTxPacket( Buffer, TX_PAYLOAD_SIZE );

        // Set led on to show transmission
        digitalWrite(PIN_LED, HIGH);

        break;
        
    default:
        break;
    }
}

