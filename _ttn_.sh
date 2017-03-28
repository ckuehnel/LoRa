#!/bin/bash

#Thingspeak
api_key='<your api key>' // Change this

if [ -f "TTN" ]
then
  JSON=`cat TTN`

  # found at http://www.christian-hoenick.com/blog/2012/01/06/shell-string-abschneiden-oder-entfernen-teilstring/ 
  TEMP=`echo ${JSON#* } | jq -r '.payload_raw' | base64 --decode`
  curl --data "api_key=$api_key&field1=$TEMP" https://api.thingspeak.com/update > log 2>&1  
  echo "" > TTN
else
  echo "No MQTT message available - try later"
fi


