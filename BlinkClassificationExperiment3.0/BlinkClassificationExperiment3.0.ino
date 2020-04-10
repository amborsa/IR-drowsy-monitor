// This code also works with five wires connected to the PCB chip that I designed.
// The hardware is just the Arduino connected with 5 wires to the Si1153 EVB, and a 4.7k pull-up resistor on the interrupt pin (D2).

#include "my_I2C_funcs.h" // functions that contain information about Si1153 registers, I2C communication packages, my own functions that read and write to Si1153
                          // registers, and global variables

// Serial output variables
byte readOuttoSerial = false;
CircularBuffer<int,1000> allCandidatesbuff;
CircularBuffer<double,1000> allSqrerrsbuff;

// timing variables
byte start_experiment = false;
int num_experiment_samples = 0;
unsigned int start_time;
unsigned int curr_time;


void setup() {
  
   delay(100); // wait 100 ms to make sure that Si1153 leaves initialization mode. (25 ms is minimum initialization time).
   Wire.begin(); // join i2c bus (allows for later i2c communication)
   Serial.begin(38400); // opening up serial communication
   
   // zeroing out the command counter to keep track of the number of successful commands
   write_reg(COMMAND, RESET_CMD_CTR);

   // setting up Si1153 global parameters   
   param_set(CHAN_LIST, 0b00000001); // sets CHAN_LIST to have only CHAN0 operational
   param_set(MEASRATE_H, 0); 
   param_set(MEASRATE_L, 2); // sets the nominal measurement rate to 625 Hz ==> measured rate for these parameters is closer to 602.5 Hz
   param_set(MEASCOUNT0, 1);
   param_set(LED1_A, 0x00); // sets a current setting for the LED at 5.5 mA

   // setting up Si1153 channel-specific parameters
   param_set(ADCCONFIG0, 0b01100010); // "short time" decimation rate, large photodiode

   // for "short time" decimation rates, measurement time is 24.4*2^HW_GAIN microseconds (x2 because of double-sampling)
   param_set(ADCSENS0, 0b00000011); // normal range, 1 accumulated sample, HW_GAIN[3:0]=n=3
   
   param_set(ADCPOST0, 0b00000000); // 16 bit measurement, no post-shift, thresholds not enabled
   param_set(MEASCONFIG0, 0b01000001); // chooses MEASCOUNT0, nominal LED current values, bank A, and enables only LED1

   // preparing interrupts on sensor
   write_reg(IRQ_ENABLE, 0b00000001); // enables interrupts on the Si1153 for channel 0

   // preparing interrupts on Arduino
   attachInterrupt(digitalPinToInterrupt(intPin), autonomous_ISR, FALLING); // enables interrupts on digital pin 2 (asserted when low)
   read_reg(IRQ_STATUS,1); // clears IRQ_STATUS so that Si1153 can send interrupts with normal behavior

   write_reg(COMMAND, START); // starting autonomous operation of the Si1153 chip
   
   // zeroing out the command counter to keep track of the number of successful commands
   write_reg(COMMAND, RESET_CMD_CTR);

   start_time = micros();
}

void loop() {

   // this block is necessary to poll the interrupt (cannot use I2C from within the ISR)
   if (irq_flag == true) {

    // /* 618 microseconds maximum
    
    // execute ALL THE TIME ==> service interrupt
    irq_flag = false; // clear the interrupt flag (on the Arduino side)
    read_3reg(three_reg); // clear the interrupt flag (on Si1153 side), read the two bytes containing proximity reading
    proxReading_current = (three_reg[1] << 8) | three_reg[2]; // combine two bytes of proximity reading data into single 2-byte short data type

    // execute ALL THE TIME ==> running data buffer and summary statistics
    if (currdataBuffsize == buffMaxsize) { // if buffer is full and is going to eliminate oldest data point, save it
      proxReading_last = proxReadings_raw.first();
    }
    buffGrowth = proxReadings_raw.push(proxReading_current); // push the current reading onto the buffer. Returns false if data was cleared at front of buffer
    if (buffGrowth == true) {
      currdataBuffsize += 1;
    }

    currMean = calc_average(currMean, buffGrowth, proxReading_current, proxReading_last, currdataBuffsize); // updates the mean of the buffer data
    currSquaredavg = calc_squared_avg(currSquaredavg, buffGrowth, proxReading_current, proxReading_last, currdataBuffsize, currMean); // updates the squared average of the buffer data
    currVar = currSquaredavg - pow(currMean,2); // updates the variance
    currStdev = pow(currVar, 0.5); // updates the standard deviation
    
    // /* 618 microseconds maximum

    // gives 10 seconds for the system to calibrate THEN executes experiment
    if (start_experiment == false) {
      curr_time = micros() - start_time;
      if (curr_time > 10000000) {
        start_experiment = true;
        Serial.print("DATA,TIME,TIMER,");
        Serial.println("----START EXPERIMENT----");
      }
    } else {
      num_experiment_samples += 1; // track number of samples recorded during experiment

      // HANDLING PRECANDIDATE BUFFER /* 50 microseconds maximum
      if (!preCandidatebuff.isEmpty()) { // updating the indices ==> NEEDS TO HAPPEN ALWAYS
        byte preCandidate_tmp = preCandidatebuff.shift();
        if (preCandidate_tmp < 50) {
          preCandidatebuff.push(preCandidate_tmp + 1);
        } else {
          indexCandidatebuff.push(preCandidate_tmp);
        }
      }
      if ((double) proxReading_current > currMean + currStdev) { // if the proximity reading is very high (probably a better way to define this)
        if (preCandidatebuff.isEmpty()) { // no competing neighbors so far
          byte larger_then_prev = true;
          for (int i=0;i<151;i++) {
            if (proxReadings_raw[currdataBuffsize - i - 2] > proxReading_current) { // makes sure there are no proximal data points that are already larger
              larger_then_prev = false;
              break;
            }
          }
          if (larger_then_prev == true) {
            preCandidatebuff.push(0); // push index to precandidate buffer
            currMax = proxReading_current; // update current maximum reading
          }  
        } else { // there are competing neighbors already
            if (proxReading_current > currMax) { // if the proximity reading is higher than neighbors
              preCandidatebuff.shift(); // remove the previous index from the precandidate buffer
              preCandidatebuff.push(0); // push the new index to the buffer
              currMax = proxReading_current; // update the current maximum reading (for later comparisons)
            }
        }
      }
      // /* 50 microseconds maximum

    // HANDLING CANDIDATE BUFFER
    if (!indexCandidatebuff.isEmpty()) {
      // /* ~20 microseconds maximum
      for (int i=0;i<indexCandidatebuff.size();i++) { // adjust the indices in the index candidate buffer
        int update = indexCandidatebuff.shift() + 1;
        indexCandidatebuff.push(update);
      }
      int peak_index = indexCandidatebuff.first();
      // /* ~20 microseconds maximum

      if (candidateHandler == 1) {
        // /* ~275 microseconds maximum
        measurement_max = (double) proxReadings_raw[currdataBuffsize - peak_index - 1];
        measurement_min = 65536;
          for (int i=0;i<201;i++) {
            double pt_measurement = proxReadings_raw[(currdataBuffsize - peak_index) - 1 - 100 + i];
            if (pt_measurement < measurement_min) {
                measurement_min = pt_measurement;
            }
            raw_measurement[i] = pt_measurement; 
          } 
          measurement_absdiff = measurement_max - measurement_min;
          candidateHandler += 1;
          // /* ~275 microseconds maximum
          
        } else if (candidateHandler == 2) { // /* all sqr_err calculations take ~1600 microseconds split into fourths
          sqrErr = 0;
          for (int i=0;i<50;i++) {
            double measurementNorm = (raw_measurement[i] - measurement_min)/(measurement_absdiff); // store normalized reading in an array
            sqrErr += pow((measurementNorm - idealBlink[i]),2);
          }
          
          candidateHandler += 1;
        } 
        else if (candidateHandler == 3) {
          for (int i=50;i<100;i++) {
            double measurementNorm = (raw_measurement[i] - measurement_min)/(measurement_absdiff); // store normalized reading in an array
            sqrErr += pow((measurementNorm - idealBlink[i]),2);
          }
          candidateHandler += 1;
        } 
        else if (candidateHandler == 4) {
          for (int i=100;i<150;i++) {
            double measurementNorm = (raw_measurement[i] - measurement_min)/(measurement_absdiff); // store normalized reading in an array
            sqrErr += pow((measurementNorm - idealBlink[i]),2);
          }     
          candidateHandler += 1;
        } 
        else if (candidateHandler == 5) {
          for (int i=150;i<201;i++) {
            double measurementNorm = (raw_measurement[i] - measurement_min)/(measurement_absdiff); // store normalized reading in an array
            sqrErr += pow((measurementNorm - idealBlink[i]),2);
          }   
          candidateHandler += 1;
        } else if (candidateHandler == 6) {
          if (sqrErr < sqrErrcutoff) {
            realBlinkbuff.push(peak_index-1); // correction for double counting
          }

          // EXPERIMENT DATA
          allCandidatesbuff.push(peak_index-1);
          allSqrerrsbuff.push(sqrErr);
          // EXPERIMENT DATA
          
          indexCandidatebuff.shift();
          candidateHandler = 1;
        }
        
      } 




      // TODO
      // HANDLING BLINK BUFFER
      if (!realBlinkbuff.isEmpty()) { // updating indices
        for (int i=0;i<realBlinkbuff.size();i++) {
          int update = realBlinkbuff.shift() + 1;
          realBlinkbuff.push(update);
        }
        if (realBlinkbuff.first() == buffMaxsize) {
          realBlinkbuff.shift();
        }
      }
      if (!allCandidatesbuff.isEmpty()) {
        for (int i=0;i<allCandidatesbuff.size();i++) {
          int update = allCandidatesbuff.shift()+1;
          allCandidatesbuff.push(update);
        }
        if (allCandidatesbuff.first() == buffMaxsize) {
          allCandidatesbuff.shift();
        }
      }
      
    }

    
    }

   // At the TERMINATION of experiment, this block is responsible for spitting all data out to Serial.
   if (num_experiment_samples == buffMaxsize && readOuttoSerial == false) {
    Serial.println("CLEARDATA");
    Serial.println("LABEL,time,timer,blinks,candidates,sqrerrs,raw");
    Serial.println("RESETTIMER");

    for (int i=0;i<buffMaxsize;i++) {
      Serial.print("DATA,TIME,TIMER,");
      if (i < realBlinkbuff.size()) {
        Serial.print(buffMaxsize - realBlinkbuff[i]);
      }
      Serial.print(",");
      if (i < allCandidatesbuff.size()) {
        Serial.print(buffMaxsize - allCandidatesbuff[i]);
      }
      Serial.print(",");
      if (i < allSqrerrsbuff.size()) {
        Serial.print(allSqrerrsbuff[i]);
      }
      Serial.print(",");
      Serial.println(proxReadings_raw[i]);

      delay(3);
    }

    readOuttoSerial = true;

    

    
   }


   
}
