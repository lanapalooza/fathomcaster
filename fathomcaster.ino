/*
  WiFi Web Server LED Blink created 25 Nov 2012 by Tom Igoe
  LED VU meter for Arduino and Adafruit NeoPixel LEDs. Written by Adafruit Industries.  Distributed under the BSD license. This paragraph must be included in any redistribution.
*/

#define N_PIXELS  25  // Number of pixels in strand
#define MIC_PIN   A0  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define DC_OFFSET  0  // DC offset in mic signal - if unusure, leave 0
#define NOISE     20  // Noise/hum/interference in mic signal
#define SAMPLES   40  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define PEAK_FALL 90  // Rate of peak falling dot


// More microphone code I don't understand
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 20,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;

// Some random numbers for the random effect
long randNumber1;
long randNumber2;
long randNumber3;
long randDelay1;
long randDelay2;
long randDelay3;

// All the stuff that makes this stuff work
#include <SPI.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <Adafruit_NeoPixel.h>

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


// A couple of arrays to order the lights in a logical more appealing look
int blue[] = {3, 9, 10, 11, 18, 24, 20, 19}; //The Blue "Candy Cane" 
int green[] = {4, 8, 12, 17, 16, 15, 14, 13}; //The Green "Candy Cane"
int leftbonus[] = {2, 1, 0}; // 3X, 4X, 5X lights on the left side
int rightbonus [] = {5, 6, 7}; // 3X, 4X, 5X lights on the right side
int whenlit [] = {21, 22, 23}; //Bigger When Lit lights at the top, two yellow, one red

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

//More WiFi stuff I don't understand
int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  Serial.begin(9600);      // initialize serial communication
  pinMode(9, OUTPUT);      // set the LED pin mode
  memset(vol, 0, sizeof(vol)); // this is used by the microphone code I don't understand
  
  strip.begin(); //NEO Pixel intializing stuff
  strip.setBrightness(255); //NEO Pixel intializing stuff
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

 
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;



  n   = analogRead(MIC_PIN);                  // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));

  }



  // Draw peak dot 
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));

   strip.show(); // Update strip

// Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL) { //fall rate

      if(peak > 0) peak--;
      dotCount = 0;
    }



  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
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
