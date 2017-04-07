/*
// This file is part of:
// Ultrasound Positioning System using the Kalman Filter
// by Enrique Fernandez (efernan@mit.edu)
// 16.322 Stochastic Estimation and Control Final Project
// Massachusetts Institute of Technology
// Fall 2013 - December 8, 2013

This Arduino sketch generates a 40 KHz signal that is sent to an ultrasound emitter. It then measures the time it takes
 for the three ultrasound receivers to capture the signal and sends those times via the serial port.
 The data is meant to be captured by a MATLAB script placed in the matlab/read_data folder.
 */


// Library found at http://forum.arduino.cc/index.php?topic=53081.msg1160999#msg1160999
// (referenced by http://playground.arduino.cc/Main/RunningMedian)

// Timer to generate 40 KHz signal as in http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
// Code as in http://www.fiz-ix.com/2012/01/how-to-configure-arduino-timer-2-registers-to-drive-an-ultrasonic-transducer-with-a-square-wave/

#define  TIMEOUT  7500
#define SAMPLE_SIZE 5
#define MEAS_PERIOD 50000 // (20 Hz sampled)
//#define MEAS_PERIOD 20000 // 10 Hz sent (50 Hz sampled)

#define ledPin 13

volatile unsigned long t1_abs, t2_abs, t3_abs;
unsigned long t1, t2, t3;
unsigned long start;
unsigned int i;

boolean done = false;

volatile boolean t1_measured;
volatile boolean measuring = false;

FastRunningMedian<unsigned long,SAMPLE_SIZE, 0> t1Median;
FastRunningMedian<unsigned long,SAMPLE_SIZE, 0> t2Median;
FastRunningMedian<unsigned long,SAMPLE_SIZE, 0> t3Median;

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(9, OUTPUT);
  //pinMode(3, INPUT);
  //startTransducer();
  // Initial values
  start = 0;
  i = 0;
  t1 = 0; t2 = 0; t3 = 0;
  t1_abs = 0; t2_abs = 0; t3_abs = 0;
  measuring = false;
  // Setup serial communication
  Serial.begin(9600);
  attachInterrupt(0, time1, RISING);
  attachInterrupt(1, time2, RISING);
  attachInterrupt(2, time3, RISING);
  // while the serial stream is not open, do nothing:
  while (!Serial) ;
  
  // Setup MATLAB communication
  // Wait for MATLAB's acknowledgement
  char rcv = 'z';
  while (rcv != 'S'){
    rcv = Serial.read();
  }
  delay(2000);
  //Send MATLAB handshake
  Serial.println('s');
  
  
//  Serial.println("Trilaterator initialized. Program about to start");
//  delay(2000);
//  Serial.println("Now, I'm ready!");
  
  //startTransducer();
  start = 0;
  
}

void loop()
{
  int recv_error;
  
  // Measure every 20 ms
  if (micros() - start >= MEAS_PERIOD){
    
 
      //Serial.println(String("Measuring input: ")+j);
      t1_measured = false;
      recv_error = 0;
      t1_abs = 0; t2_abs = 0; t3_abs = 0;
      tone(9, 40000);
      start = micros();
      
      delayMicroseconds(50);
      measuring = true;
      
      //delayMicroseconds(50);
      //noTone(9);
      
      int timeout = TIMEOUT;
      while ((t1_abs == 0 || t2_abs == 0 || t3_abs == 0)  &&  timeout > 0) {
        timeout--;
        delayMicroseconds(1);
      }
      noTone(9);
      i++;
      measuring = false;
      if (timeout > 0){
        // All three measurements received.
        recv_error = 0;
        t1 = t1_abs - start;
        t2 = t2_abs - start;
        t3 = t3_abs - start;
        t1Median.addValue(t1);
        t2Median.addValue(t2);
        t3Median.addValue(t3);
      
      } else{
        // Error receiving, set flag
        recv_error = 1;
      }
      
      // Send data always
      // Wait for MATLAB's request and send
      char rcv = 'r';
      while (rcv != 'd')
      {
        // Wait for data request
        rcv = Serial.read();
      }       
      
      //Serial.println(String(recv_error) + ", " + t1 + ", " + t2 + ", " + t3);
      Serial.println(String(recv_error) + ", " + t1Median.getMedian() + ", " + t2Median.getMedian() + ", " + t3Median.getMedian());
      //delay(2000);
    }
    else{
      delayMicroseconds(MEAS_PERIOD - (micros() - start));
    }
    

  
  
}

// Ultrasonic receivers interrupts

// Interrupt 0 INT0
// Digital Pin 3
void time1(){
   if (measuring && t1_abs == 0){
    //t1_measured = true;
    //Serial.println("t1 Detected!");
    //detachInterrupt(0);
    t1_abs = micros();
    //measuring = false;
   }
}

// Interrupt 1 INT1
// Digital Pin 2
void time2(){
   if (measuring && t2_abs == 0){
    //t1_measured = true;
    //Serial.println("t2 Detected!");
    //detachInterrupt(0);
    t2_abs = micros();
    //measuring = false;
   }
}

// Interrupt 2 INT2
// Digital Pin 0
void time3(){
   if (measuring && t3_abs == 0){
    //t1_measured = true;
    //Serial.println("t3 Detected!");
    //detachInterrupt(0);
    t3_abs = micros();
    //measuring = false;
   }
}


