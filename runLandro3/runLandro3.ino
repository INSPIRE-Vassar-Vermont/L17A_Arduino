/***************** L17A_Phenotype_10Feb2017 *****************
 *  Created 10 February, 2017
 *  Authors: Nick Livingston (nilivingston@vassar.edu)
 *
 *  As the name suggests, the purpose of this program is to
 *  instantiate the phenotype of an individual Landro, L17A
 *  agent for the evolutionary experiments to be performed
 *  using the L17A platform.  This .ino file expects a header
 *  file, which an evolutionary algorithm specifies for each
 *  individual within a population according to L17A's
 *  genotype to phenotype (GtoP) mapping algorithm.
 *
 *  A SoftwareSerial library is included in order to allow for
 *  the serial communication between the Arduino Mega 2560, and
 *  the iRobot Create 2.
 *
 *  The SPI library is needed for the Catalex Micro SD Card
 *  Adapter.  The SD library contains functions which allow for
 *  files to be created, opened, read, written to, and closed
 *  on a micro SD card that is plugged into the Catalex adapter.
 *
 *  The setup() function initailizes communication with the SD
 *  card adapter, as well as with the Create 2.  It also
 *  initializes the artificial neural network (ANN) specified
 *  by the axiliary header file.  Implicit in the structure of
 *  this network is the specification of which of L17A's sixteen
 *  sensors is to be used for this individual agent.  All
 *  sensors are permanently mounted to the chassis of L17A, and
 *  plugged into analog ports in an order that reflects the
 *  clockwise arrangment of sensors.  An individual's header file
 *  specifies which of these sensors is "grown" in that individual's
 *  development stage, as well as how it connects to the Landro's
 *  brain -- hidden and or motor (output) neurons.
 *
 *  The loop() function calls upon all other auxiliar functions
 *  to generate an agent's behavior.  This behavior includes querying
 *  all sensor ports for a given number of samples, sending the
 *  average of those samples to the ANN via the appropriate input
 *  neurons, and interpreting ANN output data into a form that can
 *  be sent to the function for controlling the Create 2 motors.  Thus,
 *  the loop() function facilitates a Landro's sense, think, act cylce.
************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include "paramsXOR719.h"

/*The baud rate change (BRC, or "baud" for short) is
 *a digitial pin that can be used for telling the iRobot
 *Create 2 to change the baud rate at which it communicates.
 *The Create 2's default baud rate is expecting a mini DIN
 *to USB connection with a full sized computer (e.g. PC).
 *Since we are communicating via an Arduino Mega 2560, we
 *want to change this baud rate to be more Arduino friendly.
 *The baud rate of choice shall be 19200, which for simplicity
 *shall also be used to perform actions when interfacing with
 *the micro SD shield.
*/
#define baudPin  11
/*There is a momentary button soldered onto L17A's breakout
 * board.  This button is wired into a digital port.  It can
 * be plugged into any digital port, the one below was
 * convenient at the time of writing this program.
 */
#define contPin  2
/* The chip select pin is a reserved pin on Arduinos
 * (pin 53 for the Mega 2560), which in this context is
 * used to initialize the Catalex SD Card Adapter.
 */
#define chipSelect 53

#define NUM_PORTS 16
#define NUM_SENSOR_SAMPLES 10

/* Minimum and maximum (operational) IR and LDR values
 * according to what L17A can sense in the extremes of its
 * environment.
 */
#define MIN_IR 0
#define MAX_IR 600 //750
#define MIN_LDR  0
#define MAX_LDR 450

/* STANDARDIZE FORMAT....
 *  ports specifies all of the ports available to L17A.
 *  All ports are specified because the fitness function
 *  requires input from all sensors, even if the ANN
 *  only uses a subset.
 */
//int analogPorts[NUM_PORTS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
int analogPorts[NUM_PORTS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
/**********************************************************/
/* This function runs through every available sensor, and
 * obtains the value for that sensor for the current timestep.
 * That value gets stored in the appropriate input node of the
 * ANN.  In essence, sense() tells the ANN about the world on a
 * given iteration.
 */
void sense();
/**********************************************************/
/* This function executes the computations of L17A's ANN.
 */
void neuralNet();
/**************************************************************/
/* This function implements the activation function for updating
 * network nodes(specifically hidden nodes).  There are two
 * slightly different formulations, one for recurrent connections
 * and one for non-recurrent connections.
 */
float activation(float value);
/**********************************************************/
/*This function records all experimentally relevant data to
 *a micro SD card.  These are the data that will be used
 *to analyze the experiment, and to compute an individual's
 *fitness.
 */
void record();
/**********************************************************/
/*This function takes two values which should range between
 * -500 and 500 (ostensibly with units of mm/sec) and sends
 * those values to the Create 2 as a sequence of four bytes
 * (two per motor), which instruct the Create 2 motors how
 * to drive.  Negative values for rightValue and leftValue
 * cause the motors to spin backward.  Larger values denote
 * faster spin.
 */
void driveMotors(int rightValue, int leftValue);
/**********************************************************/
/*This currently does not work reliably, but it spurpose is to
 * poll the front bumpers of the Create 2.  Which is to say,
 * factory made sensors, not ones installed after the fact.
 * It is somewhat unclear as to what the correct command
 * codes for reading the bumpers are, and what the proper way is
 * to specify what data subset should be read, or how it should
 * be read.
 */
int checkBumpSensors();

/**********************************************************/
/*This moves Landro according to how we want to the bump
 * response to occur. Here we just have it turn 180 and then
 * continue moving.
 */
void bumpResponse(int bumpInt);

/**********************************************************/
/*This moves Landro according to how we want to the back bump
 * response to occur. Here we just have it turn 180 and then
 * continue moving.
 */
void backBumpResponse();

/**********************************************************/
//Limits speed to a range between -500 and 500 by clipping it.
int checkSpeed(int &spd);

//NBL: Handle this differently
//Declare place for sensor values to be stored
float sensorValues[NUM_PORTS];
float sensorRawValues[NUM_PORTS];
//NBL: Handle this differently
//Delcares the bumperPort numbers and values stored
int backBumpLeft = 4;//12;
int backBumpRight = 7;//13;
int bumpRight;
int bumpLeft;
String softStr;

int firstTime = 0;
bool firstRec = true;

//https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf
/**********************************************************/
void setup() {

  //Give the iRobot Create 2 a chance to "wake up".
  delay(2000);
  //Open communication with iCreate
  pinMode(baudPin, OUTPUT);
  //Set data rate for the SoftwareSerial port, this is the iRobot's default
  Serial3.begin(19200);
  //Send three low signal pulses to the BRC pin to enact the baud rate
  //change that is specified above.
  digitalWrite(baudPin, LOW);
  delay(500);
  digitalWrite(baudPin, LOW);
  delay(500);
  digitalWrite(baudPin, LOW);
  delay(500);

  //Start robot in safe mode
  Serial3.write(128);//128 is the "start" code.
  Serial3.write(131);//Safe = 131, Full = 132
  delay(1000);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

  Serial.print("Initializing SD card...");

  //See if the SD card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return;
  }
  Serial.println("card initialized.");


  //NBL: Might not use this during evolutionary experiments.
  //But may be a way to transition from trial to trial or something.
  //Whether for a single individual, or between individuals.

  pinMode(contPin, INPUT);

  //Initialize sensor values
  for (int i = 0; i < NUM_PORTS; i++) {
    sensorValues[i] = 0;
  }
  //NBL: presumably this is a tune.
  Serial3.write(140);
    //Number 5
  Serial3.write((byte)5);
    //14 Notes
  Serial3.write((byte)1);
  Serial3.write((byte)50);
  Serial3.write((byte)16);

  //  Serial3.write(148);
  //  Serial3.write(1);
  //  Serial3.write(7);
  //NBL: setting back bumpers high?  Why?
  //Do bumper pins
  pinMode(backBumpLeft, INPUT);
  pinMode(backBumpRight, INPUT);
  digitalWrite(backBumpLeft, HIGH);
  digitalWrite(backBumpRight, HIGH);

}

/***************************************************************/
//This is the main loop of arduino code and calls everything else
// if there are issues something may be commented out here

void loop() {  
  long loopTime = millis();
  /*
  //some bump controls happen here
  int bumpInt = checkBumpSensors();
  int leftBump = digitalRead(backBumpLeft);
  int rightBump = digitalRead(backBumpRight);
  if(bumpInt != 0){
     Serial.println("Ow!");
     bumpResponse(bumpInt);
   }
  else if(leftBump==0 || rightBump==0){
     Serial.println("Ow my back!");
     Serial.print("Left bump: ");
     Serial.print(leftBump);
     Serial.print("  Right bump: ");
     Serial.println(rightBump);
     backBumpResponse();
   }*/

  int i;

  sense();
//  neuralNet();
  record();

//
//  float rmSpeed; //= preRMSPD + 0.25 * random(-250,250);
//  float lmSpeed;// = preLMSPD + 0.25 * random(-250,250);
//
//  for(i = 0; i < RMILength; i++){
//    rmSpeed += output[RMI[i]];
//  }
//
//  for(i = 0; i < LMILength; i++){
//    lmSpeed += output[LMI[i]];
//  }
//
//  Serial.println("Left Motor: " + String(scaleMotor(lmSpeed)) +
//                 " || Right Motor: " + String(scaleMotor(rmSpeed)));
//
//  driveMotors(scaleMotor(rmSpeed), scaleMotor(lmSpeed));
//
//  Serial.println("Time: " + String(millis() - loopTime));
}

/***************************************************************/
/* This function uses local variables to loop through L17A's sensors,
 * and store their current values in the "sensorValues" array.  The
 * matrix "analogPorts" names, in sequence, all 16 analog sensor ports
 * (alternating IR and LDR in a clockwise fashion, starting with the
 * former).  The port numbers correspond to the sensor positions, starting with
 * IR1 (which in broader terms is sensor 0), positioned at the exact front of
 * Landro in line with the Create 2 IR beacon sensor.  Moving clockwise, sensor 1
 * is LDR1, which sits immediately to the right of IR1 and plugs into port A1.
 * The pattern continues, and terminates with LDR8 -- sensor 15 -- in port A15.
 *
 * Each sensor is read 10 times in succession.  The average of these readings
 * will be taken to be "the" sensor value for the current iteration of
 * behavior.  This averaging is done elsewhere.  Though, sensorValues could
 * simply become a one dimensional array, each cell of which is first populated
 * by summing, and then outside of the "j" loop, averaged.
 */
void sense() {
  int i, j;
  float sensorCurrent;

  for (j = 0; j < NUM_SENSOR_SAMPLES; j++) {
    for (i = 0; i < NUM_PORTS; i++) {
    
      sensorCurrent = analogRead(analogPorts[i]);
      sensorRawValues[i] += sensorCurrent;
      if (i % 2 == 0) {
        Serial.print("IR current = ");
        Serial.println(sensorCurrent);
      }else{
        Serial.print("LDR current = ");
        Serial.println(sensorCurrent);
      }
    }
    sensorRawValues[i] = sensorRawValues[i] / NUM_SENSOR_SAMPLES;
    if (i % 2 == 0) {
      Serial.print("IR ");
      Serial.print(i);
      Serial.print(" raw = ");
      Serial.println(sensorRawValues[i]);
      sensorValues[i] = scaleIR(sensorRawValues[i]);
    }
    else {
      Serial.print("LDR ");
      Serial.print(i);
      Serial.print(" raw = ");
      Serial.println(sensorRawValues[i]);
      sensorValues[i] = scaleLDR(sensorRawValues[i]);
    }
    //sensorValues[i] = sensorValues[i] / NUM_SENSOR_SAMPLES;
    
  }

}
/***************************************************************/
/* This function executes L17A's ANN computations.  This handles
 *  all updates downstream of the inputs array.
 */
void neuralNet() {
  int h, p, o, i, s;

  //update input nodes from sensor values
  for (i = 0; i < NUM_INPUT; i++) {
    input[i] = sensorValues[sensor_to_input[i]];
    Serial.println("Input: " + String(input[i]));
  }

  //Update hidden nodes using inputs for time t
  for (h = 0; h < NUM_HIDDEN; h++) {
    hidden[h] = 0;

    //hidden from inputs
    for (i = 0; i < NUM_INPUT; i++) {
      if(input_to_hidden[i][h] != 0){
        hidden[h] += (input[i] * input_to_hidden[i][h]);
        Serial.println("Input: " + String(input[i]));
        Serial.println("Input to Hidden: " + String(input_to_hidden[i][h]));
      }
    }
    //Update hidden nodes using hidden (last) values) from time t-1
    for (p = 0; p < NUM_HIDDEN; p++) {
      if(hidden_to_hidden[p][h] != 0){
        hidden[h] += (old_hidden[p] * hidden_to_hidden[p][h]);
        Serial.println("Old Hidden: " + String(old_hidden[p]));
        Serial.println("Hidden to Hidden: " + String(hidden_to_hidden[p][h]));
      }
    }

    //Update hidden based on old motor vals
    for(o = 0; o < NUM_OUTPUT; o++){
      if(output_to_hidden[o][h] != 0){
        hidden[h] += (old_output[o] * output_to_hidden[o][h]);
      }
    }

    Serial.println("Hidden: " + String(hidden[h]));
    //Apply tanh function to total update
    hidden[h] = activation(hidden[h]);

    //Save the hidden nodes state for the future use
    old_hidden[h] = hidden[h];
    Serial.println(" Tanh Hidden: " + String(hidden[h]));
  }

  //Update output (motor) nodes based on input nodes
  for (o = 0; o < NUM_OUTPUT; o++) {
    output[o] = 0;
    
    for (i = 0; i < NUM_INPUT; i++) {
      if (input_to_output[i][o] != 0){
        output[o] += (input[i] * input_to_output[i][o]);
        Serial.println("Input: " + String(input[i]));
        Serial.println("Connect: " + String(input_to_output[i][o]));
      }
    }
    
    //Update output (motor) nodes based on hidden nodes
    for (h = 0; h < NUM_HIDDEN; h++) {
      if(hidden_to_output[h][o] != 0){
        output[o] += ( hidden[h] * hidden_to_output[h][o]);
      }
    }
    Serial.println("Output: " + String(output[o]));

    //Apply tanh equation to total update

    //NBL: Why is there an "old_output"?  And why is regular "output" just tanh, not activation?
    //Is "activation" correct?  Same activation function for hidden and output?
    old_output[o] = activation(output[o]);
    output[o] = tanh(output[o]);
    Serial.println("Output Tanh: " + String(output[o]));
  }
}
/**************************************************************/
/* This function implements the activation function for updating
 * network nodes(specifically hidden nodes).  There are two
 * slightly different formulations, one for recurrent connections
 * and one for non-recurrent connections.
 * NBL: But only one implementation is present.  Is this correct?  Should there be a second?
 *      How are we tracking recurrence, if we account for it at all?
 */
float activation(float value) {
  float update_value;
  update_value = (tanh(value - 2) + 1)/2.0;
  return update_value;
}

/***************************************************************/
/* This function writes trial information to a micro SD card.
 *  At this point, it is unclear what should be written.
 *  At the bare minimum, it should be ANN inputs and outputs.
 *  There should probably be some sort of time stamp.  Potentially,
 *  all sensors, whether used in the current agent's morphology
 *  or not, should be recorded for use in calculating fitness.
 *  Alternatively, fitness could be calculated on the fly, and
 *  piecewise fitness recorded.  That could be interesting.
 *  Then a running talley could be kept, but performance over time
 *  could also be plotted and analyzed.
 */
void record() {  
  //String to store data values
  String data = "";

  String dataName = ID + ".csv";
  
  File datafile = SD.open(dataName, FILE_WRITE);

  if(firstRec){
    firstTime = int(millis());
    datafile.println("start robot test");
    firstRec = false;
  }
  //Record raw sensor values
  for (int i = 0; i < NUM_PORTS; i++) {
    data += String(sensorRawValues[i]) + ",";
  }
  //Record output values directly from the ANN
  for (int o = 0; o < NUM_OUTPUT; o++) {
    data += String(output[o]) + ",";
  }
  //Record the time since the start of the program.
  //NBL: If we wanted to have the time since the start of the trial, we would have to record the time OF
  //the start of the trial, and do "millis() - trialStartMillis".  Probably worth doing.
  data += String(millis()) + ",";

  //data += "\n";

  //If able, write to SD card
  if (datafile) {
    //Write data to serial and SD card
    Serial.println(data);
    datafile.println(data);
    Serial.println("datalog opened succesfully");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(data);
    Serial.println("error opening datalog.txt");
  }
  //NBL: First, this is 3 minutes, not the supposed 5 minutes of a trial. Second, this shouldn't be a magic number.
  //     Last, record shouldn't necessarily be the place to keep track of this.  Though maybe it is.
   if(millis() > (300000 + firstTime)){ //Ends test after 3 minutes from first savetoSD()
      datafile.close();
      endTest();
   }

  datafile.close();
}
/**********************************************************/
/*Sends two bytes per motor.  The bytes cannot be written
 * as one continuous string, so they are broken up and
 * sent in serial.  Before sending the values, however, make
 * sure they fall between -500 and 500 by calling checkSpeed().
 */
void driveMotors(int rightValue, int leftValue) {
  //checkSpeed(rightValue);//NBL: But we have scaleMotor (which should be scaleMotor, then scaleIR, and scaleLDR.... describe the activity first, then the subject).
  //checkSpeed(leftValue);
  Serial3.write(145);//Creat 2's motor command code.
  Serial3.write(highByte(rightValue));//High and low byte for the right motor.
  Serial3.write(lowByte(rightValue));
  Serial3.write(highByte(leftValue));//High and low byte for the left motor.
  Serial3.write(lowByte(leftValue));
}

//This is somewhat complicated and I don't know exactly how it all works
//but it does so that's pretty good. It just reads the byte stream basically
//after asking for the info
//Originally sourced from http://web.ics.purdue.edu/~fwinkler/AD61600_S14/AD61600_Arduino_iRobot.pdf

/*Bumps and Wheel Drops Packet ID: 7 Data Bytes: 1, unsigned
The state of the bumper (0 = no bump, 1 = bump) and wheel drop sensors (0 = wheel raised, 1 = wheel
dropped) are sent as individual bits. */

int checkBumpSensors() {
      int bumpRight = 0;
      int bumpLeft = 0;
      char sensorbytes[5]; // variable to hold the returned 5 bytes
      Serial3.write((byte)148); // get sensor packets
      Serial3.write((byte)1);
      Serial3.write((byte)7);
      //delay(64);

      int i = 0;
      
      while (i < 5) {
        sensorbytes[i++] = 0; 
      }
      
      i = 0;
      int count = 0;
      
      while(count < 5 && Serial3.available()){
        Serial.println("waiting......");
        if(count == 0 && Serial3.available()){
          int c = Serial3.read(); 
          if(c == 19){
            sensorbytes[count] = c;
            Serial.println("Zeroth:" + String(c));
            count++;
          }
        }
        else if(count != 0 && Serial3.available()){
          int c = Serial3.read(); 
          sensorbytes[count] = c;
          Serial.println(c);
          count++;
        }
      }

      while(Serial3.available()){
        Serial3.read(); 
      }

      int sumArray = 0;
      for(i = 0; i < 5; i++){
        sumArray = sumArray + (sensorbytes[i]&0xFF);
        Serial.println("Byte: "+ String(sensorbytes[i]&0xFF));
        Serial.println("Current Sum: " + String(sumArray));
      }

      Serial.println("Sum: " + String(sumArray));
      
      bumpRight = sensorbytes[3] & 0x01;
      bumpLeft = sensorbytes[3] & 0x02;
      Serial.print("Right Bump: ");
      Serial.print(bumpRight);
      Serial.print(" Left Bump: ");
      Serial.println(bumpLeft);
      
      if(sumArray == 256){
        if(bumpRight == 1 && bumpLeft == 2){
          return 1;
        }
        else if(bumpRight == 1){
          return 2;
        }
        else if(bumpLeft == 2){
          return 3;
        }
        else{
          return 0;
        }
      }
      else{
        return 0;
      }
}


/**************************************************************/
/* If checkBumpSensors was true then this runs and ideally turns
 *  the Landro in a circle so that it is no longer stuck
 */
 
void bumpResponse(int bumpInt){
  if(bumpInt == 1){
    driveMotors(400, -400);
  }
  else if(bumpInt == 2){
    driveMotors(-400, 400);
  }
  else if(bumpInt == 3){
    driveMotors(400, -400);
    delay(200);
  }
  Serial3.write(141);
  Serial3.write(5);
  delay(200);
}

/**************************************************************/
// If the back bumpers are bumped, move away from the wall

void backBumpResponse(){
  int leftBump = digitalRead(backBumpLeft);
  int rightBump = digitalRead(backBumpRight);
  
  if(leftBump==0){
    driveMotors(400,-400);
  }
  else if(rightBump==0){
    driveMotors(-400,400);
  }
  delay(200);
}

/********************************************************/
//Just stops the robot and plays and happy tune to let you
// know the test is over. Ideally, If you press the button then you
// get out of the end test, but that needs to be worked some with Nick
void endTest() {
  while (true) {
    driveMotors(0, 0);
    Serial3.write(140);
    //Number 1
    Serial3.write((byte)1);
    //14 Notes
    Serial3.write((byte)14);
   //lE,sG,lE,sG,sE,sD,llA,
    //lD,sE,lD,sE,sD,sC,llG
    int eigth = 16;
    int quarter = 32;
    
    Serial3.write((byte)76);
    Serial3.write((byte)quarter);
    Serial3.write((byte)79);
    Serial3.write((byte)eigth);
    Serial3.write((byte)76);
    Serial3.write((byte)quarter);
    Serial3.write((byte)79);
    Serial3.write((byte)eigth);

    Serial3.write((byte)76);
    Serial3.write((byte)eigth);
    Serial3.write((byte)74);
    Serial3.write((byte)eigth);
    Serial3.write((byte)69);
    Serial3.write((byte)eigth + quarter);

    Serial3.write((byte)74);
    Serial3.write((byte)quarter);
    Serial3.write((byte)76);
    Serial3.write((byte)eigth);
    Serial3.write((byte)74);
    Serial3.write((byte)quarter);
    Serial3.write((byte)76);
    Serial3.write((byte)eigth);

    Serial3.write((byte)74);
    Serial3.write((byte)eigth);
    Serial3.write((byte)72);
    Serial3.write((byte)eigth);
    Serial3.write((byte)67);
    Serial3.write((byte)eigth + quarter);
    
    //play the song
    Serial3.write(141);
    Serial3.write((byte)1);
  }
}

void playA() {
  Serial3.write((byte)69);
  Serial3.write((byte)8);
}
void playB() {
  Serial3.write((byte)71);
  Serial3.write((byte)8);
}
void playC() {
  Serial3.write((byte)72);
  Serial3.write((byte)8);
}
void playD() {
  Serial3.write((byte)74);
  Serial3.write((byte)8);
}
void playE() {
  Serial3.write((byte)76);
  Serial3.write((byte)8);
}
void playGSharp(){
  Serial3.write((byte)68);
  Serial3.write((byte)8);
}

/********************************************************/
// Maps the tanh values from -1 to 1 to -500 to 500

float scaleMotor(float val) {
  //NBL: Do we know these are the raw inputs to the motors?
  Serial.print("scaleMotor(");
  Serial.print(val);
  Serial.println(")");
  float fromLow = -1;
  float fromHigh = 1;
  float toLow = -400;
  float toHigh = 400;
  float mapVal = (((val - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow;
  Serial.print("mapVal = ");
  Serial.print(mapVal);
  Serial.print(" map = ");
  Serial.println(map(val, fromLow, fromHigh, toLow, toHigh));
  //NBL: Also, why not use... map()?
  //mapVal = map(val, fromLow, fromHigh, toLow, toHigh);
  return (mapVal);
}

/********************************************************/
// Maps the irValue values from MIN_IR to MAX_IR to 0 to 2

float scaleIR(int val) {
  float fromLow = MIN_IR;
  float fromHigh = MAX_IR;
  float toLow = 0;
  float toHigh = 1;
  if(val < fromLow){
    val = fromLow;
  }else if(val > fromHigh){
    val = fromHigh;
  }
  float mapVal = toLow + ((val/(fromHigh - fromLow)) * (toHigh - toLow));//(((val - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow;

  return (mapVal);
}


/********************************************************/
// Maps the LDR values from MIN_LDR to MAX_LDR to 0 to 2

float scaleLDR(int val) {
  float fromLow = MIN_LDR;
  float fromHigh = MAX_LDR;
  float toLow = 0;
  float toHigh = 1;
  if(val < fromLow){
    val = fromLow;
  }else if(val > fromHigh){
    val = fromHigh;
  }
  float mapVal = toLow + ((val/(fromHigh - fromLow)) * (toHigh - toLow));//(((val - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow;

  return (mapVal);
}


