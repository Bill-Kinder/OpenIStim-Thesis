
/***********************************************************************
// 16 June 2023, Australia
// Example code for setting up stimulation current for OpenExoStim v1.0
// For any queries please email RehabExo Pty Ltd at contact@RehabExo.com
***********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AD57.h"
#include <WiFiS3.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"
#include <MQTTClient.h>
#include <CooperativeMultitasking.h>

// set your stimulation parameters
float pulse_frequency = 20;      // stimulation Pulse frequency
unsigned int off_time = 0; // off time between pulse bursts
float off_time_check = 0;
int pulse_number = 10;   // number of pulses in a burst
unsigned int pulse_width = 50;  // pulse width in microsecond
int current = 0;  // declare initial current intesity 
int calibration = 40;  // set intensity calibration value 
int duration = 0; // Stimulation session duration (seconds)

// AD5722 DAC Variables and Constants
uint8_t DacType = AD5722; // DAC hardware component number (12 bit resolution)
#define DAC_CHANNEL DAC_AB // Dual-channel
#define output_range pn108V // Bipolar, 10.8V magnitude from DAC (can also do 5V, since the Op-Amp is powered by ±48V either way.)
char slave_pin_DAC = 9; // DAC slave pin (connection to Arduino)
int16_t initial_value; // Initial value.
int16_t baseline = 2048; // Baseline 2048 (2^11)
int16_t value = baseline; // Output voltage value from DAC.

// Define the SS1306 display connected to I2C pins
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define LED_BLINK 12 // LED to blink during stimulation
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Define rotary encoder pins
#define ENC_A 3
#define ENC_B 2

uint16_t intensity = 0; // stimulation intensity - read from rotary encoder.
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 2500;
int _fastIncrement = 10;
char buffer[64];  // buffer must be big enough to hold all the message

// Wi-Fi Constants and Objects

// SSID, Password - defined in "arduino_secrets.h"
char wifi_ssid[] = SECRET_SSID;
char wifi_pass[] = SECRET_PASS;

// Parameters password - defined in "arduino_secrets.h"
char param_pass[] = PARAMS_PASS;

// Wi-Fi client object
WiFiClient wifiClient;

// MQTT Constants and Objects

// MQTT client buffer size - 256 bytes
#define mqtt_buffer_size 256

// MQTT Client object.
MQTTClient mqttClient = MQTTClient(mqtt_buffer_size);

// Broker address, port, qos, retain flag.
const char broker[] = "test.mosquitto.org";
int port = 1883;
int qos = 1;
boolean retain = true;

// MQTT Topics declarations - same as in GUI code.

// Topics: From GUI, to Stimulator.
const char topic_params_stim[] = "stim/params";
const char topic_test_stim[] = "stim/test";
const char topic_cmd[] = "stim/cmd";
const char topic_ping_stim[] = "stim/ping";

// Topics: From Stimulator, to GUI
const char topic_data[] = "gui/data";
const char topic_test_gui[] = "gui/test";
const char topic_ping_gui[] = "gui/ping";
const char topic_feedback[] = "gui/feedback";
const char topic_params_gui[] = "gui/params";

int test_count_msg; // Sent to "gui/test" in response to a message from "stim/test"
int ping_count_msg; // Sent to "gui/ping" from stim once every 5 seconds.

const char MQTT_CLIENT_ID[] = "stimulator-1"; // Identifier for mqtt client (optional)

// Json objects

// Outgoing - data (ping message) and params (get params command)
JsonDocument outgoing_data_msg;
JsonDocument outgoing_params_msg;

// Incoming - set params command.
JsonDocument incoming_params_msg;

// Message buffer for sending messages to GUI.
char messageBuffer[512];

// Variable for debugging purposes (increments each stimulation cycle).
int num_cycles;

// Timer variables for keeping track of stimulation progress.
unsigned long timer_value = 0; // Set to millis() when a stimulation starts.
unsigned long current_time = 0; // Set to millis() when checking if stimulation should continue, then "current_time - timer_value > duration" is checked.

// Timer variables for seeing the actual pulse width from the DAC.
unsigned long pulse_start = 0;
unsigned long pulse_end = 0;

// Flag to check and control if the stimulation should continue.
bool currently_stimulating;

// Multitasking objects
// Main tasks - tasks.run called in main program loop.
CooperativeMultitasking tasks;

// Continuations - these are functions which are executed during tasks.loop.
Continuation stimulate; // Stimulate pulses
Continuation stimulation_should_continue; // Check if current stimulation has exceeded duration.
Continuation mqttCheckConnection; // Check if connected to MQTT Broker
Continuation ping_to_gui; // Send a ping message to the GUI.

// Guard - a function used to test a boolean condition.

Guard mqtt_is_connected; // Check if still connected to MQTT Broker. Differs from mqtt.connected() method as it can be used in tasks.after, which requires a Guard-type object.

// Measurement pins for telemetry - voltage at current sensing resistor outputs.
// The arduino has a resolution of 4.9mV per analog unit. For example, a reading of 512 on the analog input would imply the output voltage is 2.5088V.
#define V_ref1_pin A0
#define V_ref2_pin A1
int v_ref1 = 0;
int v_ref2 = 0;

void setup() {
  // Initialise timer value to 0 (prevent inaccurate data transmission).
  outgoing_data_msg["timer_value"] = 0;
  pinMode(LED_BLINK, OUTPUT);
  // initialize the DAC
  AD57.debug(0);
  Serial.println("Starting DAC");
  // initialise the AD57xx and connections to and
  // from the AD57xx
  init_AD57xx();
  // set max count depending on AD57xx : 12, 14 or 16 bits
  if (DacType == AD5752)    initial_value = 0xffff;
  else if (DacType == AD5732) initial_value = 0x3fff;
  else if (DacType == AD5722) initial_value = 0x0fff; // This is the type used in this specific implementation of the stimulator. 0x0fff corresponds to 4095.
  
  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  // Start the serial monitor to show output
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  // Begin Wi-Fi client and MQTT client
  wifiSetup();
  mqttSetup();
  // Just in case there are left over messages saying to start a stimulation, set the currently_stimulating flag to false. Also set number of cycles to 0 (this is the setup and no stimulation should start until the user clicks "Start Stimulation in the GUI")
  currently_stimulating = false;
  num_cycles = 0;
}

//**Main program**// - Similar to window loop function in GUI code.
void loop() {
  // Run the tasks - stimulate, send message, process data etc. 
  tasks.run();
  // Run the MQTT client - i.e.: Listen for messages, execute callback functions.
  mqttClient.loop();
}

void stimulate(){
  // This function outputs digital pulses to the DAC for purposes of providing  voltage pulses to the Op-Amp. Note: There is always a 1 millisecond delay when setting values on the DAC (See the AD57.h library code)
  // Check if stimulation should be continuing.
  if(currently_stimulating){
    // generate stimulation pulses
    for (int i=0; i<pulse_number; i++) {
      // Set pulse start time.
      pulse_start = micros();
      // positive phase
      if(intensity >= baseline){ // Prevent overflow (Voltage magnitude reverses after 12 bits)
        intensity = baseline - 1;
      }
      // intensity ranges between 0 and 2047, so value ranges between 2048 and 4095. [2048, 4095] for a 12-bit DAC corresponds to [0, Vo], where Vo is 5V, 10V or 10.8V (see output_range constant above).
      value = baseline + intensity;

      // Positive Phase

      // Set channel voltage to value - this command currently delays the program by 1 millisecond, causing a 1ms bias in the output pulse. Perhaps the library can be modified, as it currently uses a 1ms clock rate.
      AD57.setDac(DAC_CHANNEL, value);
      // delay for the pulse width (e.g.: 20µs)
      delayMicroseconds(pulse_width);
      // record pulse end time, print time difference --> output pulse width (in microseconds).
      pulse_end = micros();
      // Serial.println(pulse_end - pulse_start); Debugging purposes

      // Negative Phase - same as positive phase, except use negative value (-value).
      AD57.setDac(DAC_CHANNEL, -value);
      delayMicroseconds(pulse_width);
    }
    // Finished pulses - set to 0 for the remainder of the period.

    AD57.setDac(DAC_CHANNEL, 0);

    digitalWrite(LED_BLINK, !digitalRead(LED_BLINK)); // toggle LED  

    // Display screen code - currently commmented out as it was not soldered on during testing.

    // print the value to serial display
    // display.clearDisplay();
    // display.setTextSize(2);
    // display.setTextColor(WHITE);
    // display.setCursor(0, 0);

    // sprintf(buffer, "intensity = %d mA", current);
    // display.println(buffer);
    // display.display();
    // Serial.print("intensity = ");
    // Serial.print(intensity);
    
    num_cycles += 1; // For pulse_frequency counting purposes

    // If stimulation duration has not been exceeded, wait for the remained of the Pulse period, then repeat this function again. Otherwise, do nothing, and end the stimulation pulses.

    if(currently_stimulating){
      tasks.after(off_time, stimulate);

    }else{
      Serial.println("Stopping stimulation cycle.");
    }
  }
  // End
}

void stimulation_should_continue(){
  // Check if the stimulation should be continued.
  // Extra if statement check at the start for robustness.
  if(currently_stimulating){
    current_time = millis() - timer_value; // Update timer.
    if(current_time > duration){
      currently_stimulating = false; // Update flag to false if duration has been exceeded. Then exit the function.
    }else{
      currently_stimulating = true; // Retain flag at true value if duration not exceeded, then check again in 1 second.
      tasks.after(1000, stimulation_should_continue);
      Serial.println("Stimulation should continue."); // 
    }
  }else{
    Serial.println("Stimulation should not continue."); // 
    Serial.println("stimulation has ended.");
  }
}

void ping_to_gui(){
  // Send a ping message to the GUI once every five seconds. All that is sent is an integer which continuously increments.
  ping_count_msg += 1;
  // Convert integer to character, place in message buffer.
  itoa(ping_count_msg, messageBuffer, 10);
  // Publish message to ping topic.
  mqttClient.publish(topic_ping_gui, messageBuffer);
  // Repeat after 5 seconds if the mqtt client is still connected.
  tasks.ifForThen(mqtt_is_connected, 5000, ping_to_gui);
}

bool mqtt_is_connected(){
  // Check if client is still connected. Used specifically for ping timing.
  return mqttClient.connected();
}

void(* reset_program) (void) = 0; // Call this if the program gets stuck. A bit glitchy - causes errors.

//**Interrupt service routine**// - called when turning the rotary encoder on the device.
void read_encoder() {

  // Encoder interrupt routine for both pins. Updates intensity
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update intensity if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 10;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    intensity = intensity + changevalue;              // Update intensity
    encval = 0;
  }
  else if( encval < -3 ) {  // Four steps backward
    int changevalue = -10;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    if (intensity > 0){
      intensity = intensity + changevalue;              // Update intensity
    }
    else{
      intensity = 0;
    }
    encval = 0;
  }
    // Calcluate and display set current intesity value
  current = intensity / calibration;
  outgoing_data_msg["Current (mA)"] = current;
}

void wifiSetup(){
  // Function for connecting the Arduino to Wi-Fi. wifi ssid and password can be defined in "arduino_secrets.h"
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifi_ssid);
  int retry_count = 0;
  while (WiFi.begin(wifi_ssid, wifi_pass) != WL_CONNECTED) {
    // failed to connect, delay and retry after 1 second until connected.
    if(retry_count >= 10){
      // may cause error, not recommended
      // reset_program();
    }
    Serial.print(".");
    delay(1000);
    retry_count += 1;
  }
  Serial.println("You're connected to the network");
  printWifiStatus();
  Serial.println();
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
}

void mqttSetup(){
  // Begin the MQTT client (requires successful wifi connection).
  mqttClient.begin(broker, port, wifiClient);
  // Set the callback function for handling messages.
  mqttClient.onMessage(messageHandler);
  // Try connect to the broker.
  mqttTryConnect();
}

void subscribeToTopic(const char topic_name[]){
  // Subscribe to a given topic, print the result. If the result is failure, the device may need to be manually reset via the reset button. Alternatively, the function can be called again (recursive function until successful connection established).
  if (mqttClient.subscribe(topic_name)){
    Serial.print("Successfully subscribed to the topic: ");   
  }
  else{
    Serial.print("Failed to subscribe to the topic: ");
    // Recursive implementation - can be commented out but usually gets stuck in a loop - MQTT client never succeeds in subscribing.
    // Serial.println(topic_name);
    // Serial.println("Trying again...");
    // subscribeToTopic(topic_name); // Recursive re-try.

    // Reset the program if subscription fails (may cause error, not recommended)
    // Serial.println("Subscription failed, resetting program.");
    // reset_program();

    Serial.println("Trying MQTT connection again.");
    mqttTryConnect();
  }
  Serial.println(topic_name);
}

void mqttTryConnect(){
  // Attempt to connect to the MQTT broker. If successful, subscribe to relevant topics (see above function).
  Serial.println("Connecting to MQTT broker");
  int retry_count = 0;
  while(!mqttClient.connect(MQTT_CLIENT_ID)){
    if(retry_count >= 10){
      // reset program - may cause error, not recommended
      // reset_program();

    }
    Serial.print(".");
    retry_count += 1;
    delay(1000);
  }
  subscribeToTopic(topic_test_stim);
  subscribeToTopic(topic_params_stim);
  subscribeToTopic(topic_cmd);
  subscribeToTopic(topic_ping_stim);

  // Success message
  Serial.println("Arduino - MQTT broker Connected!");
  // Ping to GUI immediately to notify that Arduino has successfully connected to the Broker.
  tasks.now(ping_to_gui);
  // After 30 seconds, check connection status.
  tasks.after(30000, mqttCheckConnection);
}

void mqttCheckConnection(){
  // Check if still connected to the MQTT Broker. If not, attempt to connect again.
  if(!mqttClient.connected()){
    Serial.println("MQTT not connected, attempting reconnection.");
    mqttTryConnect();
  }else{ // If still connected, print a message to the serial monitor then check again after 30 seconds.
    Serial.println("MQTT still connected.");
    tasks.after(30000, mqttCheckConnection);
  }
}

void messageHandler(String &topic, String &payload) {
  // Callback function called when the MQTT client receives a message.
  // First print the topic and payload, then, decide what to do based on the topic.
  Serial.println("Arduino - received from MQTT:");
  Serial.println("- topic: " + topic);
  Serial.println("- payload:");
  Serial.println(payload);

  if(topic == topic_ping_stim){
    // "stim/ping", sent once per second from GUI to Arduino. Arduino response: Send data to the GUI, to be displayed in the "data frame".
    // Voltage at current sensors - analog read [0, 1024] corresponding to [0, 5V] - can be converted to current.
    v_ref1 = analogRead(V_ref1_pin);
    v_ref2 = analogRead(V_ref2_pin);

    outgoing_data_msg["timer_value"] = current_time/1000;
    outgoing_data_msg["Current (mA)"] = current; // current = intensity / calibration, however this is currently arbitrary and does not represent the true value of output current. The true value is v1/10Ω. However due to hardware limitations during testing, the true values of v1 and v2 were not obtained.
    outgoing_data_msg["v1"] = v_ref1;
    outgoing_data_msg["v2"] = v_ref2;
    outgoing_data_msg["currently_stimulating"] = currently_stimulating;
    outgoing_data_msg["num_cycles"] = num_cycles;

    // Serialise the Json dictionary into the message buffer then publish the message under topic "gui/data"
    Serial.print("Sending message: ");
    serializeJson(outgoing_data_msg, messageBuffer);
    Serial.println(messageBuffer);
    mqttClient.publish(topic_data, messageBuffer);
  }

  else if(topic == topic_cmd){
    // "stim/cmd" command sent from GUI to Arduino via buttons.
    // Check the payload to see the specific command, then go from there.
    if(payload == "start stimulation"){
      // Command: Start stimulation. Only "start" if not currently stimulating.
      if(currently_stimulating){
        // Already stimulating - Publish feedback message to the GUI. 
        mqttClient.publish(topic_feedback, "Arduino Feedback: Stimulation already started.");
      }
      else{
        // Start stimulation
        Serial.println("Starting stimulation");

        // Publish feedback message to the GUI.
        mqttClient.publish(topic_feedback, "Arduino Feedback: Starting stimulation now.");

        // Reset number of cycles to 0.
        num_cycles = 0;

        // Change currently stimulating flag to true.
        currently_stimulating = true;
        
        // Reset timer.
        timer_value = millis();

        // Call stimulate and stimulation_should_continue function.
        tasks.now(stimulate);
        tasks.now(stimulation_should_continue);
      }
    }
    else if(payload == "stop stimulation"){
      // Command: Stop stimulation. Only "stop" if currently stimulating.
      if(currently_stimulating){
        // Stop stimulation
        Serial.println("Stopping stimulation");
        
        // Publish feedback message to the GUI.
        mqttClient.publish(topic_feedback, "Arduino Feedback: Stopping stimulation now.");

        // Change currently stimulating flag to false. This will automatically stop the stimulations, which check this flag before doing anything.
        currently_stimulating = false;

      }
      else{
        // Already stopped stimulating.
        Serial.println("Already stopped stimulation.");

        // Publish feedback message to the GUI.
        mqttClient.publish(topic_feedback, "Arduino Feedback: Stimulation already stopped.");
      }
    }
    else if(payload == "get parameters"){
      // Command: Get parameters.
      // Send the current parameters of the Arduino to the GUI. Useful if the GUI is closed while the stimulator is active, then re-opened.
      Serial.println("Sending current parameters");

      // Set values in outgoing_params Json dictionary.
      outgoing_params_msg["Pulse Frequency (Hz)"] = pulse_frequency;
      outgoing_params_msg["Pulse Width (\u00b5s)"] = pulse_width;
      outgoing_params_msg["Duration (s)"] = duration/1000;
      outgoing_params_msg["Intensity"] = intensity;
      outgoing_params_msg["Calibration"] = calibration;
      outgoing_params_msg["Pulses per Burst"] = pulse_number; 

      // Serialise Json dictionary into message buffer, publish to gui.
      serializeJson(outgoing_params_msg, messageBuffer);
      mqttClient.publish(topic_params_gui, messageBuffer);
    }

    else if(payload == "get data"){
      // Alternative to sending data while pinging - not currently used but can be implemented in GUI with a "get data" button.
      Serial.println("Sending data");
      // Send data
      v_ref1 = analogRead(V_ref1_pin);
      v_ref2 = analogRead(V_ref2_pin);
      outgoing_data_msg["timer_value"] = current_time;
      outgoing_data_msg["Current (mA)"] = current;
      outgoing_data_msg["v1"] = v_ref1;
      outgoing_data_msg["v2"] = v_ref2;
      outgoing_data_msg["currently_stimulating"] = currently_stimulating;
      outgoing_data_msg["num_cycles"] = num_cycles;
      Serial.print("Sending message: ");
      serializeJson(outgoing_data_msg, messageBuffer);
      Serial.println(messageBuffer);
      mqttClient.publish(topic_data, messageBuffer);
    }

    else if (payload == "reset program"){
      Serial.println("Resetting program");
      // May cause error, not recommended
      // reset_program();
    }
  }

  else if(topic == topic_params_stim){
    // "stim/params" - Changing parameters message. Will only work if the password is correct.
    // Deserialise message payload into a Json dictionary structure.
    deserializeJson(incoming_params_msg, payload);
    Serial.println("Received new parameter settings.");

    // Check password. If password is incorrect, send a rejection feedback message. If password is correct, modify the parameters then send a success feedback message.
    if(incoming_params_msg["Password"] != param_pass){ // If password is incorrect, do not modify parameters
      Serial.println("Incorrect password. Parameters will not be modified.");
      mqttClient.publish(topic_feedback, "Arduino Feedback: Incorrect password. Parameters will not be modified.");
    }else{ // If password is correct, modify the parameters
      Serial.println("Modifying parameters");

      // Pulse frequency
      pulse_frequency = incoming_params_msg["Pulse Frequency (Hz)"];                     
      Serial.print("New pulse_frequency = ");
      Serial.println(pulse_frequency);

      // Pulses per burst
      pulse_number = incoming_params_msg["Pulses per Burst"]; 
      Serial.print("New pulse_number = ");  
      Serial.println(pulse_number);

      // Pulse width
      pulse_width = incoming_params_msg["Pulse Width (\u00b5s)"];   
      Serial.print("New pulse_width = ");   
      Serial.println(pulse_width);

      // Calibration
      calibration = incoming_params_msg["Calibration"];   
      Serial.print("New calibration = ");   
      Serial.println(calibration);

      // Intensity
      intensity = incoming_params_msg["Intensity"];       
      Serial.print("New intensity = ");     
      Serial.println(intensity);

      // Duration
      duration = incoming_params_msg["Duration (s)"];    
      Serial.print("New duration = ");      
      Serial.println(duration);
      // Convert to milliseconds
      duration = 1000*duration;
      Serial.println();    
      
      // Calculate current based on intensity and calibration value.
      current = intensity / calibration;
      // Calculate off time between pulse bursts (milliseconds) - needs to account for the time length of the pulse train make the pulse frequency work as intended.
      // Note - this does not account for the 1ms delay in DAC (which should be fixable - maybe using port manipulation instead of the DAC library?)

      off_time_check = 1000/pulse_frequency - (2*(float)pulse_width*pulse_number)/1000.0;
      // Feedback message.
      if(off_time_check < 1){ // Less than 1 millisecond.
        off_time = 0; // Set to 0 milliseconds.
        mqttClient.publish(topic_feedback, "Arduino Feedback: Parameters modified successfully, but off time was below 0. It has been set to 0, so you may not receive the desired pulse frequency.");
      }else{
        off_time = (unsigned int)off_time_check;
        mqttClient.publish(topic_feedback, "Arduino Feedback: Parameters modified successfully.");
      }

    }
  }

  else if(topic == topic_test_stim){
    // "stim/test" - when GUI "Send Test Message" button is clicked. Reply with an incrementing integer - test_count_msg.
    Serial.println("Test message received, sending feedback message.");
    test_count_msg += 1;
    itoa(test_count_msg, messageBuffer, 10);
    mqttClient.publish(topic_test_gui, messageBuffer);
  }
}

// Below: Functions from OpenIStim for controlling the AD57 DAC.

void init_AD57xx(){
  // set output voltage range in binary
  // if you want to test 2scomp, see remark 1 in AD57.h, use
  // AD57.begin(DAC_CHANNEL, output_range, COMP2); 
  AD57.begin(DAC_CHANNEL, output_range);

  Serial.print("get range ");
  Serial.println(AD57.getRange(DAC_CHANNEL));

  // enable therminal shutdown and overcurrent protection
  // also stop CLR as, starting from 0v after OP_CLEAR op_code)
  // this can be changed for full negative after clear with
  // AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|SET_CLR_SET));
  AD57.setControl(OP_INSTR,(SET_TSD_ENABLE |SET_CLAMP_ENABLE|STOP_CLR_SET));
   
  // clear the DAC outputs
  AD57.setControl(OP_CLEAR);
}

bool check_status(bool disp){
    uint16_t stat = AD57.getStatus();
    bool ret = 0;
    
    if (stat & stat_err_TO)
    {
      Serial.println(F("Therminal overheat shutdown"));
      ret = 1;
    }
    
    if (stat & stat_err_CA)
    {
      Serial.println(F("DAC - A overcurrent detected"));
      ret = 1;
    }

    if (stat & stat_err_CB)
    {
      Serial.println(F("DAC - B overcurrent detected"));
      ret = 1;
    }

    if (disp == 1)
    {
      Serial.println(F("Display settings\n"));
      
      if (stat & stat_TS)
         Serial.println(F("Therminal shutdown protection enabled"));
      else
         Serial.println(F("Therminal shutdown protection disabled"));

      if (stat & stat_CLR)
        Serial.println(F("DAC output set to midscale or fully negative with OP_CLEAR command"));
      else
        Serial.println(F("DAC output set to 0V with OP_CLEAR command"));
      
      if (stat & stat_CLAMP)
         Serial.println(F("Overcurrent protection enabled"));
      else
         Serial.println(F("Overcurrent protection disabled"));

      if (stat & stat_SDO) // you will never see this one :-)
         Serial.println(F("Serial data out disabled"));
      else          
         Serial.println(F("Serial data out enabled"));

      Serial.println();
    }

    return(ret);
}

void serialTrigger(String message){
  Serial.println();
  Serial.println(message);
  Serial.println();

  while (!Serial.available());

  while (Serial.available())
    Serial.read();
}

/* errorLoop loops forever.*/
void errorLoop(){
  // power-off both channels 
  AD57.setPower(STOP_PUA|STOP_PUB);
  
  Serial.print(F("Critical error, looping forever"));
  for (;;);
}