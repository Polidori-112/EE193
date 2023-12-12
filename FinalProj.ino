// Arduino Brain Library - Brain Serial Test

// Description: Grabs brain data from the serial RX pin and sends CSV out over the TX pin (Half duplex.)
// More info: https://github.com/kitschpatrol/Arduino-Brain-Library
// Author: Eric Mika, 2010 revised in 2014
#include <SoftwareSerial.h>
#include <Brain.h>

String nums[10] = {
    "1111110",
    "0110000",
    "1101101",
    "1111001",
    "0110011",
    "1011011",
    "1011111",
    "1110000",
    "1111111",
    "1110011"
  };


int in1pin = 11; //PWM pin
int in2pin = 12; // non PWM pin

// Set up the brain parser, pass it the hardware serial object you want to listen on.
SoftwareSerial mySerial(0, 1); // RX, TX
Brain brain(mySerial);

void setup() {
    // Start the hardware serial.
    Serial.begin(9600);
    mySerial.begin(9600);
    for (int i = 2; i <= 8; i++) {
    pinMode(i, OUTPUT);
  }
  setPinsFromBinaryString("1101001");

  pinMode(in1pin, OUTPUT);
  pinMode(in2pin, OUTPUT);
}

void setPinsFromBinaryString(String binaryString) {
  // Check if the string length is 7
  if (binaryString.length() == 7) {
    // Iterate through each character in the string
    for (int i = 0; i < 7; i++) {
      // Convert the character to an integer (either 0 or 1)
      int value = binaryString.charAt(i) - '0';

      // Set the corresponding pin (pins 2-8) to the value
      digitalWrite(i + 2, value);
    }
  } else {
    // Handle the case where the string length is not 7 (optional)
    Serial.println("Invalid string length. Please provide a 7-digit string.");
  }
}

int extractValueBetweenCommas(String inputString) {
  int commaIndex1 = inputString.indexOf(',');  // Find the first comma
  int commaIndex2 = inputString.indexOf(',', commaIndex1 + 1);  // Find the second comma
  int commaIndex3 = inputString.indexOf(',', commaIndex2 + 1);

  // Check if both commas are found
  if (commaIndex2 != -1 && commaIndex3 != -1) {
    // Extract the substring between the first and second commas
    String valueString = inputString.substring(commaIndex1 + 1, commaIndex2);

    // Convert the substring to an integer
    int extractedValue = valueString.toInt();

    return extractedValue;
  } else {
    // Handle the case where the required commas are not found
    Serial.println("Error: Two commas not found in the input string.");
    return 0;  // Return a default value (you can modify this based on your requirements)
  }
}

void loop() {
    // Expect packets about once per second.
    // The .readCSV() function returns a string (well, char*) listing the most recent brain data, in the following format:
    // "signal strength, attention, meditation, delta, theta, low alpha, high alpha, low beta, high beta, low gamma, high gamma"
    String values;
    if (brain.update()) {
        values = (brain.readCSV());
        Serial.println(extractValueBetweenCommas(values));
        Serial.println((values));
        setPinsFromBinaryString(nums[(int)(extractValueBetweenCommas(values)/15)]);

        int attention = extractValueBetweenCommas(values);
        if((attention/15) > 4)
        {
          digitalWrite(in1pin, HIGH);
          digitalWrite(in2pin, LOW);
        }
        else
        {
          digitalWrite(in1pin, LOW);
          digitalWrite(in2pin, HIGH);
        }
    }
}
