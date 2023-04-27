#include <WiFi.h>
#include <Adafruit_GFX.h>  // Include core graphics library for the display.
#include <Adafruit_SSD1306.h>  // Include Adafruit_SSD1306 library to drive the display.
#include <Adafruit_BME280.h> //For BME280 module
#include "ThingSpeak.h"
#include "Adafruit_INA219.h"
#include <PID_v1.h>

//Wi-Fi Setup
const char* ssid = "Xiaomi 11T";   // your network SSID (name)
const char* password = "Bokinet1";   // your network password
WiFiClient  client;

//ThingSpeak channel and API key
unsigned long myChannelNumber = 1907191;
const char * myWriteAPIKey = "IL62361NNH9W5X53";
const char * myReadAPIKey = "422X0OVXEU1CXSJC";

// For motor RPM calculation
unsigned long lastTime = 0;
unsigned long timerDelay = 20000;
const byte PulsesPerRevolution = 2;  // Set how many pulses there are on each revolution. Default: 2.
const unsigned long ZeroTimeout = 320000;  // For high response time, a good value would be 100000.
const byte numReadings = 4;  // Number of samples for smoothing. The higher, the more smoothing,  Default: 2.
volatile unsigned long LastTimeWeMeasured;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.
volatile unsigned long PeriodAverage = ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total
unsigned long FrequencyRaw;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal;  // Frequency without decimals.
unsigned long RPM;  // Raw RPM without any processing.
unsigned int PulseCounter = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before 
unsigned long PeriodSum; // Stores the summation of all the periods to do the average.
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;  // Stores the last time we measure a pulse in that cycle.
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned long readings[numReadings];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.
char string[4];  // Create a character array of 4 characters
char stringg[3];  // Create a character array of 3 characters
char SetPo[4];

//FOR PWM
const int PWM_Pin = 19;  /* GPIO16 OUTPUT*/
uint16_t DutyCycle;
const int PWMFreq = 5000; /* 5 KHz */
const int PWMChannel = 0;
const int PWMResolution = 12;
const int ADC_RESOLUTION = 4095; /* 12-bit */

// Define the PID parameters
int dutyCycle;
double SetpointPom;
double Setpoint, Input, Output;
double Kp = 0.0027, Ki = 0.065, Kd = 0.0025;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//FOR CURRENT
#define CurrentPin 39 //GPIO for Current measurement
int Amps = 0;
float AvgAcs;

//For Voltage
#define VoltagePin 34 //GPIO for Voltage measurement
#define FILTER_LEN  5
uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int VoltagePin_Raw;
int AN_Pot1_Filtered = 0;
float analog_voltage=0.0;
int voltage_offset = 0;

//For Temperature
float temperatureC;

//For Working Modes
unsigned int autos=2;
int automode = 2;
unsigned int setpoints=0;
unsigned long lastTime1 = 0;
unsigned long timerDelay1 = 150;

// Create objects
Adafruit_SSD1306 display(128, 64);  // Create display.
Adafruit_BME280 bme; //BME280 connect to ESP32 I2C (GPIO 21 = SDA, GPIO 22 = SCL) 
Adafruit_INA219 ina219;
TaskHandle_t Task1;

//Initialize function
void initBME(){
if (!bme.begin(0x76)) {
Serial.println("Could not find a valid BME280 sensor, check wiring!");
while (1); } }

void initINA(){
if (!ina219.begin()) {
Serial.println("Failed to find INA219 chip");
while (1); } }

void Pulse_Event(){
PeriodBetweenPulses = micros() - LastTimeWeMeasured;
LastTimeWeMeasured = micros();
if(PulseCounter >= AmountOfReadings){
PeriodAverage = PeriodSum / AmountOfReadings;
PulseCounter = 1;
PeriodSum = PeriodBetweenPulses;
int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
AmountOfReadings = RemapedAmountOfReadings;}
else{PulseCounter++; PeriodSum = PeriodSum + PeriodBetweenPulses;} }

void Task1code( void * pvParameters) {
Serial.print("Task1 running on core ");
Serial.println(xPortGetCoreID());
for(;;) {
if((millis() - lastTime) > timerDelay) {
float averagee=average;
if(Amps<0){Amps=0;}
int autos = ThingSpeak.readFloatField(myChannelNumber, 3, myReadAPIKey);
int setpoints = ThingSpeak.readFloatField(myChannelNumber, 4, myReadAPIKey);
ThingSpeak.setField(8,analog_voltage);
ThingSpeak.setField(5,Amps);
ThingSpeak.setField(6,temperatureC);
ThingSpeak.setField(7,averagee);
if(temperatureC > 31){
ThingSpeak.setStatus("Pažnja! Visoka temperatura");}
int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
if(x == 200){
digitalWrite(2, HIGH);
Serial.println("ThingSpeak channel update successful!");
if(autos==1 && setpoints!=0){
Setpoint=setpoints;
automode=1; }
if(autos==2){
Setpoint=SetpointPom;
automode=2;}
if(autos==0 || setpoints==0) {
autos=automode;
setpoints=Setpoint;
digitalWrite(2, LOW);} }
else{Serial.println("ThingSpeak problem updating channel. HTTP error code " + String(x));
digitalWrite(2, LOW);}
lastTime = millis(); }  } }

void setup(){
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize display with the I2C address of 0x3C.
Serial.begin(115200);  // Begin serial communication.
initINA();
initBME();
WiFi.begin(ssid, password);
Serial.println("\nConnecting");
while(WiFi.status() != WL_CONNECTED){
Serial.print(".");
delay(300);}
Serial.println("\nConnected to the WiFi network");
Serial.print("Local ESP32 IP: ");
Serial.println(WiFi.localIP());
// Initialize ThingSpeak
ThingSpeak.begin(client);
delay(300);
ledcSetup(PWMChannel, PWMFreq, PWMResolution);
ledcAttachPin(PWM_Pin, PWMChannel);   
myPID.SetMode(AUTOMATIC);  // Set the PID controller to automatic mode
pinMode(2, OUTPUT);
pinMode(25, OUTPUT);
pinMode(26, OUTPUT);
pinMode(27, OUTPUT);
//Display static fields
display.clearDisplay();
display.setCursor(16,0);
display.setTextSize(1);
display.setTextColor(WHITE);
display.println("PARAMETRI MOTORA");
display.println("");
display.println("Rezim: ");
display.println("Set Point:     /min.");
display.println("Br.obrtaja:    /min.");
display.print("Napon: ");
display.setCursor(71,40);
display.println("V");
display.print("Struja: ");
display.setCursor(65,48);
display.println("mA");
display.print("Temperatura: ");
display.setCursor(110,56);
display.cp437(true);
display.write(248);
display.print("C");
display.display();
xTaskCreatePinnedToCore(
Task1code, /* Function to implement the task */
"Task1", /* Name of the task */
10000,  /* Stack size in words */
NULL,  /* Task input parameter */
2,  /* Priority of the task */
&Task1,  /* Task handle. */
1); /* Core where the task should run */
attachInterrupt(18, Pulse_Event, RISING);
Serial.println("Setup complete!!!");}

//ADC Noise Reduction By Multi-Sampling & Moving Average Digital Filtering
uint32_t readADC_Avg(int ADC_Raw){
int i = 0;
uint32_t Sum = 0;
AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
if(AN_Pot1_i == FILTER_LEN){
AN_Pot1_i = 0;}
for(i=0; i<FILTER_LEN; i++){
Sum += AN_Pot1_Buffer[i];}
return (Sum/FILTER_LEN);}

void loop(){
VoltagePin_Raw = analogRead(VoltagePin);
AN_Pot1_Filtered = readADC_Avg(VoltagePin_Raw);
analog_voltage = map(AN_Pot1_Filtered,0,4095,0,840) + voltage_offset;
analog_voltage /= 100;
digitalWrite(25, LOW);
digitalWrite(27, HIGH);
//RPM Calculation
LastTimeCycleMeasure = LastTimeWeMeasured;
CurrentMicros = micros();
if(CurrentMicros < LastTimeCycleMeasure){
LastTimeCycleMeasure = CurrentMicros;}
FrequencyRaw = 10000000000 / PeriodAverage;
if(PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra){
FrequencyRaw = 0.1;
//ZeroDebouncingExtra = 2000;
ZeroDebouncingExtra = 4000;}
else{ZeroDebouncingExtra = 0;}
FrequencyReal = FrequencyRaw / 10000;
RPM = FrequencyRaw / PulsesPerRevolution * 60;
RPM = RPM / 10000;
total = total - readings[readIndex];
readings[readIndex] = RPM;
total = total + readings[readIndex];
readIndex = readIndex + 1;
if(readIndex >= numReadings){
readIndex = 0;}
average = total / numReadings;
Input = map(average,0,3399,0,3399);
myPID.Compute();
dutyCycle = map(Output, 0, 255, 0, 3399);
if(automode==1){  
ledcWrite(PWMChannel, dutyCycle);}
if(automode==2){
SetpointPom=map(analogRead(32),0,4095,0,3399);
Setpoint=SetpointPom;
ledcWrite(PWMChannel, dutyCycle);}
if(temperatureC >= 24.5){
digitalWrite(26, HIGH);}
else if (temperatureC < 24.5){
digitalWrite(26, LOW);}
AnalogFilter();
if((millis() - lastTime1) > timerDelay1) {
displayData();
lastTime1 = millis(); } }

void AnalogFilter(){
unsigned int y=0;
float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
for (int y = 0; y < 20; y++){ //Get 150 samples 
AcsValue=ina219.getCurrent_mA();
Samples = Samples + AcsValue;  //Add samples together
delay (5);}
AvgAcs=Samples/20.0;
Amps=AvgAcs;
temperatureC = bme.readTemperature();
float Raw = analogRead(32);} //Motor speed potentiometer raw value

void displayData(){
display.setCursor(38,16);
if(automode == 2){
display.print("Rucno");}
else if (automode == 1){
display.print("Auto.");}
dtostrf(Setpoint, 4, 0, SetPo);
display.setTextColor(WHITE,BLACK);
display.setCursor(65,24);
display.print(SetPo);
dtostrf(average, 4, 0, string);
display.setTextColor(WHITE,BLACK);
display.setCursor(65,32);
display.print(string);
display.setCursor(40,40);
display.print(analog_voltage);
display.setCursor(45,48);
dtostrf(Amps, 3, 0, stringg);
display.print(stringg);
display.setCursor(75,56);
display.print(temperatureC);
display.display(); }
