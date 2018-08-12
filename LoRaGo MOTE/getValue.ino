// This function simulates getting a sensor value

void getValue()
{
  float temperatureC = (float) random(25,35); // this simulates a temperature value between 20 and 30 degrees         
  char result[10];                            // Buffer big enough for 7-character float        
  
  dtostrf(temperatureC, 6, 1, result);        // Leave room for too large numbers!
  strcat(result, " *C");

  memcpy(mydata, result, strlen(result));

  if (DEBUG)
  {
    Serial.print("Measured temperature = ");
    Serial.print(temperatureC); 
    Serial.println(" *C"); 
  }
}
