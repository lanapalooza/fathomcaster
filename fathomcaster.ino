/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will print the IP address of your WiFi module (once connected)
  to the Serial monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 9.

  If the IP address of your board is yourAddress:
  http://yourAddress/H turns the LED on
  http://yourAddress/L turns it off

  This example is written for a network using WPA encryption. For
  WEP or WPA, change the Wifi.begin() call accordingly.

  Circuit:
   Board with NINA module (Arduino MKR WiFi 1010, MKR VIDOR 4000 and UNO WiFi Rev.2)
   LED attached to pin 9

  created 25 Nov 2012
  by Tom Igoe
*/

#define LED_PIN    6 // Pin the NeoPixel's are on
#define LED_COUNT 25 // Number of NeoPixel's configured
#define MIC_PIN   A0  // Microphone is attached to this analog pin
#define SAMPLE_WINDOW   5  // Sample window for average level (in MS) for microphone
byte peak = 20;      // For microphone/LED code Peak level of column; used for falling dots
unsigned int sample; // For microphone/LED code
byte dotCount = 0;  // For microphone/LED code - Frame counter for peak dot
byte dotHangCount = 0; // For microphone/LED code - Frame counter for holding peak dot


long randNumber1;
long randNumber2;
long randNumber3;
long randDelay1;
long randDelay2;
long randDelay3;

#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <Adafruit_NeoPixel.h>

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// I've got 25 LED's in my project, but they are not in a logical order for what I'm looking to accomplish so I am putting them into an array for ordering
int blue[] = {3, 9, 10, 11, 18, 24, 20, 19};
int green[] = {4, 8, 12, 17, 16, 15, 14, 13};

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  Serial.begin(9600);      // initialize serial communication
  pinMode(9, OUTPUT);      // set the LED pin mode

  strip.begin();
  strip.setBrightness(255);
  strip.show(); // Initialize all pixels to 'off';

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status

  // first block 
  current_code_H();
}

char current_code = 0;

void current_code_H(void)
{
  for (int i = 0; i <= 7; i++) {
      strip.setPixelColor(green[i], 200, 200, 200);
      strip.setPixelColor(blue[i], 200, 200, 200);
      strip.show();
      delay(100);
      //Serial.println(currentLine);
    }
  for (int i = 7; i >= 0; i--) {
    strip.setPixelColor(green[i], 0, 0, 0);
    strip.setPixelColor(blue[i], 0, 0, 0);
    strip.show();
    delay(100);
  }
}

void current_code_I(void)
{
  for (int i = 0; i <= 24; i++) {
      strip.setPixelColor(i, 200, 200, 200);
      strip.show();
      delay(100);
      //Serial.println(currentLine);
    }
  for (int i = 24; i >= 0; i--) {
    strip.setPixelColor(i, 0, 0, 0);
    strip.show();
    delay(100);
  }
}

void current_code_J(void)
{
    randNumber1 = random(0, 24);
    randNumber2 = random(0, 24);
    randNumber3 = random(0, 24);
    randDelay1 = random(50, 150);
    randDelay2 = random(50, 150);
    randDelay3 = random(50, 150);
    strip.setPixelColor(randNumber1, 200, 200, 200);
    strip.show();
    delay(randDelay1);
    strip.setPixelColor(randNumber2, 200, 200, 200);
    strip.show();
    delay(randDelay2);
    strip.setPixelColor(randNumber3, 200, 200, 200);
    strip.show();
    delay(randDelay3);

    strip.setPixelColor(randNumber1, 0, 0, 0);
    strip.show();
    delay(randDelay1);
    strip.setPixelColor(randNumber2, 0, 0, 0);
    strip.show();
    delay(randDelay2);
    strip.setPixelColor(randNumber3, 0, 0, 0);
    strip.show();
    delay(randDelay3);
    
}

void current_code_L(void)
{
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;

  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(MIC_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  int lights = (peakToPeak / 1024) * 125;
  int previousLights;
  previousLights = lights;
  if (lights >= 8)
  {
    lights = 8;
  }
  else
  {
    lights = lights;
  }

  //  Serial.print ("peakToPeak: ");
  //  Serial.println(peakToPeak);
  Serial.print ("lights: ");
  Serial.println(lights);
  //  Serial.print ("previousLights: ");
  // Serial.println(previousLights);

  for (int i = 0; i <= lights; i++) {
    strip.setPixelColor(green[i], 200, 200, 200);
    strip.setPixelColor(blue[i], 200, 200, 200);
    /* Serial.print("UP i: ");
      Serial.println(i);
      Serial.print("UP j: ");
      Serial.printl n(j); */
    strip.show();
    delay(50);
  }
  for (int i = lights; i >= 0; i--) {
    strip.setPixelColor(green[i], 0, 0, 0);
    strip.setPixelColor(blue[i], 0, 0, 0);
    /*Serial.print("DOWN i: ");
      Serial.println(i);
      Serial.print("DOWN j: ");
      Serial.println(j); */
    strip.show();
    delay(50);
  }
}

void loop() { 
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the Fathomcaster on Simple Candy Cane<br>");
            client.print("Click <a href=\"/I\">here</a> turn the Fathomcaster on Snake<br>");
            client.print("Click <a href=\"/J\">here</a> turn the Fathomcaster on Random<br>");
            client.print("Click <a href=\"/L\">here</a> turn the Fathomcaster on Microphone<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":



        if (currentLine.endsWith("GET /H")) {

          current_code = 1;         
          //current_code_H();
        }

        if (currentLine.endsWith("GET /I")) {

          current_code = 2;         
          //current_code_H();
        }

        if (currentLine.endsWith("GET /J")) {

          current_code = 3;         
          //current_code_H();
        }

        if (currentLine.endsWith("GET /L")) {

          current_code = 4;         
          //current_code_L();
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
  else
  {
    if (current_code == 1)
    {
      current_code_H();
    }
    else if (current_code == 2)
    {
      current_code_I();
    }
    else if (current_code == 3)
    {
      current_code_J();
    }
    else if (current_code == 4)
    {
      current_code_L();
    }
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
