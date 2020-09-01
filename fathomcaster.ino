/*
  WiFi Web Server LED Blink created 25 Nov 2012 by Tom Igoe
  LED VU meter for Arduino and Adafruit NeoPixel LEDs. Written by Adafruit Industries.  Distributed under the BSD license. This paragraph must be included in any redistribution.
*/

#define N_PIXELS  27  // Number of pixels in strand
#define MIC_PIN   A1  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define BTN_PIN    8  // BTN pin
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
int blue[] = {5, 11, 12, 13, 20, 26, 22, 21}; //The Blue "Candy Cane"
int green[] = {6, 10, 14, 19, 18, 17, 16, 15}; //The Green "Candy Cane"
int leftbonus[] = {4, 3, 2}; // 3X, 4X, 5X lights on the left side
int rightbonus [] = {7, 8, 9}; // 3X, 4X, 5X lights on the right side
int whenlit [] = {23, 24, 25}; //Bigger When Lit lights at the top, two yellow, one red

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

//More WiFi stuff I don't understand
int status = WL_IDLE_STATUS;
WiFiServer server(80);

#define COUNT_ATTEMPTS_CONNECT_WIFI 2    // every 5 sec 
char wifi_enabled = 1;                  // default wifi is enabled

// Part of tuner
#define TUNE_LED_YELLOW_HIGH 23
#define TUNE_LED_YELLOW_LOW  24
#define TUNE_LED_RED         25
// LEDS to lit number of string
int string_led_number[6] = {7, 8, 9, 4, 3, 2};

#define E1 329.63
#define B2 246.94
#define G3 196.00
#define D4 146.83
#define A5 110.00
#define E6 82.41

float freq[6] = {E1, B2, G3, D4, A5, E6};

#define sampleFrequency_float   3125.0
#define sampleFrequency_integer 3125
#define bufferSize 1024

volatile int rawData[bufferSize] ;  // Buffer for ADC capture
volatile int sampleCnt = 0;                    // Pointer to ADC capture buffer
long currentSum, previousSum, twoPreviousSum ;
int threshold = 0;
float frequency = 0;
byte pdState = 0;

// ADC
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}


uint32_t adcSetupFast1(void)
{
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;      // Gain select as 1X
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;   //  1.6 V Supply VDDANA

  // Set sample length and averaging
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  ADCsync();
  ADC->SAMPCTRL.reg = 0x0A;  ; //sample length in 1/2 CLK_ADC cycles Default is 3F

  //Control B register
  int16_t ctrlb = 0x420;       // Control register B hibyte = prescale, lobyte is resolution and mode
  ADCsync();
  ADC->CTRLB.reg =  ctrlb     ;
  adcReadFast1();  //Discard first conversion after setup as ref changed
}

uint32_t adcReadFast1() {

  ADCsync();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[MIC_PIN].ulADCChannelNumber; // Selection for the positive ADC input

  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared

  ADCsync();
  ADC->SWTRIG.bit.START = 1;                // Start ADC conversion

  while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Wait till conversion done
  ADCsync();
  uint32_t valueRead = ADC->RESULT.reg;

  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable the ADC
  ADCsync();
  ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
  return valueRead;
}

void all_leds_off(void)
{
  for (int i = 2; i < N_PIXELS; i++)
  {
    strip.setPixelColor(i, 0, 0, 0);
  }
    strip.show();
}

#define MODE_MIN 1
#define MODE_MAX (4+6)
char current_code = 0; //Variable to keep track of which effect I want to show
char previous_code = 0;

void check_button(void)
{
  static char last_state = 1;  // not pressed (input = VCC pullup)
  char current_state = digitalRead(BTN_PIN);

  // was pressed (input = GND)
  if (current_state == 0 && last_state == 1)
  {
    current_code ++;
    if (current_code > MODE_MAX)
      current_code = MODE_MIN;
  }
  // memorize previous state
  last_state = current_state;
}

// TUNER PART

void setup_tuner(void)
{
  // for pitch detector
  timer_doPrepare(4, sampleFrequency_integer);
}

void findFrequency(int current_string)
{
  // lit string number
  for (int i = 0; i < 6; i++)
  {
    if (i == current_string)
      strip.setPixelColor(string_led_number[i], 200, 200, 200);
    else
      strip.setPixelColor(string_led_number[i], 0, 0, 0);
  }
  strip.show();

  // 10% erro maybe
  float freq_min1 = freq[current_string] - (freq[current_string] / 10);
  float freq_max1 = freq[current_string] + (freq[current_string] / 10);

  // 5% error good result
  float freq_min2 = freq[current_string] - (freq[current_string] / 20);
  float freq_max2 = freq[current_string] + (freq[current_string] / 20);

  sampleCnt = 0;

  timer_doStart(4);
  while (sampleCnt < bufferSize); // wait for data
  timer_doStop(4);

  // Calculate mean to remove DC offset
  long meanSum = 0 ;

  // char str[80];
  //for (int k = 0; k < bufferSize; k += 8) {
  //  sprintf(str, "%d %d %d %d", rawData[k], rawData[k+2], rawData[k+4], rawData[k+6]);
  //  Serial.println(str);
  //}

  for (int k = 0; k < bufferSize; k++) {
    meanSum += rawData[k] ;
  }
  int mean = meanSum / bufferSize ;

  //Serial.println("mean = ");
  //Serial.println(mean);
  // Remove mean
  for (int k = 0; k < bufferSize; k++) {
    rawData[k] -= mean ;
  }

  // Amplitude
  /*
    long skv = 0;
    for (int k = 0; k < bufferSize; k++)
    {
      skv += (int)rawData[k] * (int)rawData[k];
    }
    skv = sqrt(skv / bufferSize);

    sprintf(str, "TEST1 amplitude =  %d", skv);
    Serial.println(str);
    return;
  */

  // Autocorrelation
  currentSum = 0 ;
  pdState = 0 ;
  for (int i = 0; i < bufferSize && (pdState != 3); i++) {
    // Autocorrelation
    float period = 0;
    twoPreviousSum = previousSum ;
    previousSum = currentSum ;
    currentSum = 0 ;
    for (int k = 0; k < bufferSize - i; k++) {
      currentSum += int(rawData[k]) * int(rawData[k + i]) ;
    }
    // Peak detection
    switch (pdState) {
      case 0:   // Set threshold based on zero lag autocorrelation
        threshold = currentSum / 2 ;
        pdState = 1 ;
        break ;
      case 1:   // Look for over threshold and increasing
        if ((currentSum > threshold) && (currentSum - previousSum) > 0) pdState = 2 ;
        break ;
      case 2:   // Look for decreasing (past peak over threshold)
        if ((currentSum - previousSum) <= 0) {
          // quadratic interpolation
          float interpolationValue = 0.5 * (currentSum - twoPreviousSum) / (2 * previousSum - twoPreviousSum - currentSum) ;
          period = i - 1 + interpolationValue ;
          pdState = 3 ;
        }
        break ;
      default:
        pdState = 3 ;
        break ;
    }

    // Frequency identified in Hz
    if (threshold > 100 && period > 0)
    {
      frequency = sampleFrequency_float / period;

      if (frequency < 500)
      {
        Serial.println(frequency);

        if (frequency >= freq_min1 && frequency <= freq_max1)
        {
          if (frequency >= freq_min2 && frequency <= freq_max2)
          {
            strip.setPixelColor(TUNE_LED_RED, 200, 200, 200);
            strip.setPixelColor(TUNE_LED_YELLOW_HIGH, 0, 0, 0);
            strip.setPixelColor(TUNE_LED_YELLOW_LOW, 0, 0, 0);
          }
          else if (frequency >= freq_min1 && frequency < freq_min2)
          {
            strip.setPixelColor(TUNE_LED_RED, 0, 0, 0);
            strip.setPixelColor(TUNE_LED_YELLOW_HIGH, 0, 0, 0);
            strip.setPixelColor(TUNE_LED_YELLOW_LOW, 200, 200, 200);
          }
          else if (frequency > freq_max2 && frequency <= freq_max1)
          {
            strip.setPixelColor(TUNE_LED_RED, 0, 0, 0);
            strip.setPixelColor(TUNE_LED_YELLOW_HIGH, 200, 200, 200);
            strip.setPixelColor(TUNE_LED_YELLOW_LOW, 0, 0, 0);
          }
          strip.show();
          return;
        }
      }
    }
  }
  strip.setPixelColor(TUNE_LED_RED, 0, 0, 0);
  strip.setPixelColor(TUNE_LED_YELLOW_HIGH, 200, 200, 200);
  strip.setPixelColor(TUNE_LED_YELLOW_LOW, 0, 0, 0);
  strip.show();
}


// LIGHT EFFECTS PART

//Simple "Candy Cane" effect, where the LED's walk the green and blue strings
void current_code_H(void)
{
  all_leds_off();
  
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

//Simple effect where the LED's natrually progress through in order as I laid them out on the guitar, it ended up looking pretty cool, I'm calling it "snake" as that is what i reminds me of when it moves through the LED's
void current_code_I(void)
{
  for (int i = 2; i <= 26; i++) {
    strip.setPixelColor(i, 200, 200, 200);
    strip.show();
    delay(100);
    //Serial.println(currentLine);
  }
  for (int i = 28; i >= 2; i--) {
    strip.setPixelColor(i, 0, 0, 0);
    strip.show();
    delay(100);
  }
}

//This effect feels more like a pinball attract mode, not any real logicall grouping of lights, just randomly lighting 3 up for random amounts of time.
void current_code_J(void)
{
  randNumber1 = random(2, 26);
  randNumber2 = random(2, 26);
  randNumber3 = random(2, 26);
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

// The real reason to light this thing up in the first place, read the microphone, I went through a couple of different ones by the MAX4466 is the one I've landed on, the microphone is epoxied into the flipper hole. I basically have no idea how or why this works, the comments are not mine, thank Adafruit for this one.
void current_code_L(void)
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  n   = adcReadFast1();                  // Raw reading from mic
  n   = abs(n - 512 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if (height < 0L)       height = 0;     // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top


  // Color pixels based on rainbow gradient
  for (i = 2; i < N_PIXELS; i++) {
    if (i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i, Wheel(map(i, 0, strip.numPixels() - 1, 30, 150)));

  }

  // Draw peak dot
  if (peak > 0 && peak <= N_PIXELS - 1) strip.setPixelColor(peak, Wheel(map(peak, 0, strip.numPixels() - 1, 30, 150)));

  strip.show(); // Update strip

  // Every few frames, make the peak pixel drop by 1:

  if (++dotCount >= PEAK_FALL) { //fall rate

    if (peak > 0) peak--;
    dotCount = 0;
  }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if (++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl)      minLvl = vol[i];
    else if (vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)

}

// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
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

// TIMER PART

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

void timer_doSetFrequency(int tcNum, int frequencyHz)
{
  TcCount16* TC;
  if (tcNum == 3)
  {
    TC = (TcCount16*) TC3;
  }
  else if (tcNum == 4)
  {
    TC = (TcCount16*) TC4;
  }

  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void timer_doStart(int tcNum)
{
  TcCount16* TC;
  if (tcNum == 3)
  {
    TC = (TcCount16*) TC3;
  }
  else if (tcNum == 4)
  {
    TC = (TcCount16*) TC4;
  }

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void timer_doStop(int tcNum)
{
  TcCount16* TC;
  if (tcNum == 3)
  {
    TC = (TcCount16*) TC3;
  }
  else if (tcNum == 4)
  {
    TC = (TcCount16*) TC4;
  }

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void timer_doPrepare(int tcNum, int frequencyHz)
{
  TcCount16* TC;
  if (tcNum == 3)
  {
    TC = (TcCount16*) TC3;
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  }
  else if (tcNum == 4)
  {
    TC = (TcCount16*) TC4;
    REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TC4_TC5)) ;
  }

  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  timer_doSetFrequency(tcNum, frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  if (tcNum == 3)
    NVIC_EnableIRQ(TC3_IRQn);
  else if (tcNum == 4)
    NVIC_EnableIRQ(TC4_IRQn);
}

void TC3_Handler()
{
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we chec button
  if (TC->INTFLAG.bit.MC0 == 1)
  {
    TC->INTFLAG.bit.MC0 = 1;

    check_button();
  }
}

void TC4_Handler()
{
  TcCount16* TC = (TcCount16*) TC4;
  // If this interrupt is due to the compare register matching the timer count
  // we chec button
  if (TC->INTFLAG.bit.MC0 == 1)
  {
    TC->INTFLAG.bit.MC0 = 1;

    if (sampleCnt < bufferSize)
    {
      rawData[sampleCnt] = adcReadFast1();//adcReadFast();//analogRead(MIC_PIN); ;
      sampleCnt++ ;
    }
  }
}

void setup() {
  Serial.begin(9600);      // initialize serial communication
  pinMode(LED_BUILTIN, OUTPUT);     // set the board LED pin mode
  pinMode(BTN_PIN, INPUT_PULLUP); // set the BTN pin mode
  memset(vol, 0, sizeof(vol)); // this is used by the microphone code I don't understand

  strip.begin(); //NEO Pixel intializing stuff
  strip.setBrightness(255); //NEO Pixel intializing stuff
  strip.show(); // Initialize all pixels to 'off';
  strip.setPixelColor(0,255,255,255);
  strip.setPixelColor(1,255,255,255);
  strip.show();
  
  // After powering BTN pressed -> wifi will be ignored
  if (digitalRead(BTN_PIN) == 0)
  {
    wifi_enabled = 0;
  }
  else
  {
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE)
    {
      Serial.println("Communication with WiFi module failed!");
      // don't continue
      //while (true);

      // continue w/o wifi
      wifi_enabled = 0;
    }
  }

  if (wifi_enabled == 1)
  {
    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println("Please upgrade the firmware");
    }
    // attempt to connect to Wifi network:
    for (char i = 0; i < COUNT_ATTEMPTS_CONNECT_WIFI; i++)
    {
      Serial.print("Attempting to connect to Network named: ");
      Serial.println(ssid);                   // print the network name (SSID);

      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);

      // Connect success -> exit
      if (status == WL_CONNECTED)
      {
        server.begin();                           // start the web server on port 80
        printWifiStatus();                        // you're connected now, so print out the status

        break;
      }
      else
      {
        // It was last attempt and connect status is bad
        if (i == (COUNT_ATTEMPTS_CONNECT_WIFI - 1))
        {
          wifi_enabled = 0;
        }
        else
        {
          // wait 5 seconds for next attempt
          delay(5000);
        }
      }
    }
  }

  //
  adcSetupFast1();

  // tuner setup
  setup_tuner();

  // first block
  current_code_H();

  // use hardware timer for button , loop() is so busy by strip and WIFI
  timer_doPrepare(3, 10); // 10 Hz = 100 ms tick
  timer_doStart(3);
}

void loop() {
  // work with wifi
  if (wifi_enabled == 1)
  {
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
              // for tuner
              client.print("Click <a href=\"/S1\">here</a> turn the Fathomcaster string 1<br>");
              client.print("Click <a href=\"/S2\">here</a> turn the Fathomcaster string 2<br>");
              client.print("Click <a href=\"/S3\">here</a> turn the Fathomcaster string 3<br>");
              client.print("Click <a href=\"/S4\">here</a> turn the Fathomcaster string 4<br>");
              client.print("Click <a href=\"/S5\">here</a> turn the Fathomcaster string 5<br>");
              client.print("Click <a href=\"/S6\">here</a> turn the Fathomcaster string 6<br>");

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

          // Check to see what the client request was eg. "GET /H" or "GET /L" etc:

          if (currentLine.endsWith("GET /H"))
            current_code = 1;
          else if (currentLine.endsWith("GET /I"))
            current_code = 2;
          else if (currentLine.endsWith("GET /J"))
            current_code = 3;
          else if (currentLine.endsWith("GET /L"))
            current_code = 4;
          else if (currentLine.endsWith("GET /S1"))
            current_code = 5;
          else if (currentLine.endsWith("GET /S2"))
            current_code = 6;
          else if (currentLine.endsWith("GET /S3"))
            current_code = 7;
          else if (currentLine.endsWith("GET /S4"))
            current_code = 8;
          else if (currentLine.endsWith("GET /S5"))
            current_code = 9;
          else if (currentLine.endsWith("GET /S6"))
            current_code = 10;

        }
      }
      // close the connection:
      client.stop();
      Serial.println("client disonnected");
    }
  }

  if (current_code != previous_code)
    all_leds_off();

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
  else if (current_code >= 5 && current_code <= 10)
  {
    // call function with arg 0..5
    //check_tuner(current_code - 5);
    findFrequency(current_code - 5);
  }

  previous_code = current_code;
}
