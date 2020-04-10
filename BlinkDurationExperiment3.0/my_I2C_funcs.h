#include <Wire.h> // necessary for I2C communication
#include <math.h> // for powers and basic mathematical functionality
#include "Si1153.h" // specifies the Si1153 I2C address and the register addresses
#include "CircularBuffer.h" // useful for circular buffer functionality

// for timing
unsigned int mytime;

// defining global variables
byte intPin = 5; // defines D2 to be the interrupt pin on the Arduino.
byte three_reg[3]; // to store IRQ_STATUS, HOSTOUT0, HOSTOUT1
short proxReading_current; // to store the 2-byte proximity reading

int buffCounter = 0;


#define buffMaxsize 36240 // maximum size of the proximity reading buffer (how long data is saved) ==> 60 seconds @ 604 Hz
CircularBuffer<short,buffMaxsize> proxReadings_raw; // to store 1-minute of proximity readings
int currdataBuffsize = 0; // to store size of buffer
short proxReading_last = 0; // to store the proximity reading about to be ejected from the buffer
byte buffGrowth = 1; // acts as a boolean to indicate whether the buffer grew or retained its maximal size
double currMean = 0; // average value of proximity reading data, initialized to zero for an empty buffer
double currSquaredavg = 0; // squared average of proximity reading data (for calculation of standard deviation)
double currVar; // variance of the proximity reading data
double currStdev; // standard deviation of the proximity reading data

CircularBuffer<byte,1> preCandidatebuff;
short currMax = 0;

CircularBuffer<int,500> indexCandidatebuff;
byte candidateHandler = 1;
double sqrErrcutoff = 10;
double measurement_min;
double measurement_max;
double measurement_absdiff;
double sqrErr;
double raw_measurement[201];

CircularBuffer<int,500> realBlinkbuff;
//int blinkbufflocation = 0; //NOT IN USE RIGHT NOW
//double smoothed_waveform[201];

//CircularBuffer<double,500> durationBuff;
//CircularBuffer<int,500> durationIndexBuff;



volatile bool irq_flag = false; // initialized interrupt flag

// function to read a certain register in the Si1153
// address is the address of the register internal to the Si1153, num_data is the number of bytes requested (usually should be 1)
byte read_reg(byte address, int num_data) {
  byte data = 0xFF; // if nothing happens, data should have this value

  // this block of code tells the Si1153 which internal address to provide data for
  Wire.beginTransmission(IR_ADDRESS); // begins transmission with the Si1153
  Wire.write(address); // writes the address that the uC wants to read from
  Wire.endTransmission(); //

  Wire.requestFrom(IR_ADDRESS, num_data);      // request a certain number of bytes from the register

  while (Wire.available()) // waits for entire read and saves it
  {
    data = Wire.read();
  }

  return data; // returns the read byte
}

// function to read 3 registers in Si1153 (IRQ_STATUS, HOSTOUT0, HOSTOUT1) ==> single read as opposed to reading three bytes separately
void read_3reg(byte *pdata) {
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(IRQ_STATUS);
  Wire.endTransmission();

  Wire.requestFrom(IR_ADDRESS, 3);

  while (Wire.available()) {
    pdata[0] = Wire.read();
    pdata[1] = Wire.read();
    pdata[2] = Wire.read();
  }
}

// address is the address of the register internal to the Si1153, val is the value to actually be written
void write_reg(byte address, byte val) {  // Write to a register
  Wire.beginTransmission(IR_ADDRESS);
  Wire.write(address);     
  Wire.write(val);       
  Wire.endTransmission();     
}

// function that reads value at a certain location in the paramter table
// param_address is the address of the parameter within the parameter table
byte param_query(byte param_address) { // Read the value in the parameter table at a certain address
  
  byte cmd_ctr_initial; // initializing local variables
  byte cmd_ctr_current;
  byte response0;
  byte cmd_err;

  response0 = read_reg(RESPONSE0, 1); // read the RESPONSE0 register which contains cmd_ctr and cmd_err
  cmd_ctr_initial = response0&0x0F; // mask to just have the bottom four bits
  cmd_ctr_current = cmd_ctr_initial; // the current cmd_ctr value is the initial one for now

  write_reg(COMMAND, PARAM_QUERY|param_address); // command to query a parameter

  // waits until the command has been successfully performed (cmd_ctr iterates) or if there is an error
  while (cmd_ctr_initial != (cmd_ctr_current - 1)) {
    response0 = read_reg(RESPONSE0, 1); // read the RESPONSE0 register which contains cmd_ctr and cmd_err
    cmd_err = response0&0x10; // mask to just have the fifth bit
    cmd_ctr_current = response0&0x0F;

    if (cmd_err == 0x10) {
      return 0xFF; // error
    }
  }
  
  byte val = read_reg(RESPONSE1, 1); // read the queried value
  return val; // no error
}

// function that sets a paramter in the parameter table
// param_address is the address of the parameter within the parameter table and val is the value to actually be written
byte param_set(byte param_address, byte val) { // Set a parameter in the parameter table at a certain address
  
  byte cmd_ctr_initial; // initializing local variables
  byte cmd_ctr_current;
  byte response0;
  byte cmd_err;
  
  write_reg(HOSTIN0, val); // writes a value to register in the Si1153

  response0 = read_reg(RESPONSE0, 1); // read the RESPONSE0 register which contains cmd_ctr and cmd_err
  cmd_ctr_initial = response0&0x0F; // mask to just have the bottom four bits
  cmd_ctr_current = cmd_ctr_initial; // the current cmd_ctr value is the initial one for now

  write_reg(COMMAND, PARAM_SET|param_address); // writes from that register to the parameter table

  // waits until the command has been successfully performed (cmd_ctr iterates) or if there is an error
  while (cmd_ctr_initial != (cmd_ctr_current - 1)) {
    response0 = read_reg(RESPONSE0, 1); // read the RESPONSE0 register which contains cmd_ctr and cmd_err
    cmd_err = response0&0x10; // mask to just have the fifth bit
    cmd_ctr_current = response0&0x0F;

    if (cmd_err == 0x10) {
      return 0xFF; // error
    }
  }

  return 0; // no error
}

void autonomous_ISR() {
  irq_flag = true; //set the flag that indicates an interrupt has been received
}

// function that calculates the current average in a buffer (iteratively)
double calc_average(double prevMean, byte buffIsgrowing, short newReading, short ejectedReading, int currBuffsize) {
  double currMean;
  if (buffIsgrowing == false) { // if buffer is not growing
    currMean = prevMean + ((double) newReading)/currBuffsize - ((double) ejectedReading)/currBuffsize; // subtract the ejected value AND add new value (typecasting gave
                                                                                                     // me some trouble here for a while)
  } else {
    currMean = (currBuffsize*prevMean + newReading)/(currBuffsize+1); // add new value to the mean calculation
  }
  return currMean;
}

// function that calculates the current squared sum of the data over the data length (\Sigma_{i=0}^N x_i^2)/N (iteratively)
// this is analogous to the calc_average function
double calc_squared_avg(double prevSquaredave, byte buffIsgrowing, short newReading, short ejectedReading, int currBuffsize, double currMean) {
  double currSquaredave;
  if (buffIsgrowing == false) {
    currSquaredave = prevSquaredave + pow((double) newReading,2)/currBuffsize - pow((double) ejectedReading,2)/currBuffsize; 
  } else {
    currSquaredave = (currBuffsize*prevSquaredave + pow((double) newReading,2))/(currBuffsize+1);
  }
  return currSquaredave;
}


// an "ideal" blink waveform
double idealBlink[201] = {0.012790146849834212, 0.0123164377072469, 0.008526764566556141, 0.011842728564659585, 0.005684509711037428, 0.007579346281381514, 0.0123164377072469, 0.010895310279487543, 0.01847465656087164, 0.006631927996209471, 0.009474182851728186, 0.008526764566556141, 0.006631927996209471, 0.005684509711037428, 0.008053055423968829, 0.011842728564659585, 0.0023685457129314, 0.008526764566556141, 0.009474182851728186, 0.011842728564659585, 0.0, 0.008526764566556141, 0.006631927996209471, 0.013737565135006256, 0.013263855992418942, 0.009947891994315498, 0.007579346281381514, 0.005684509711037428, 0.009947891994315498, 0.011369019422074855, 0.001894836570344086, 0.008053055423968829, 0.011842728564659585, 0.010421601136900228, 0.008526764566556141, 0.0023685457129314, 0.006631927996209471, 0.009000473709140871, 0.002842254855518714, 0.010421601136900228, 0.006631927996209471, 0.0047370914258628, 0.010421601136900228, 0.010895310279487543, 0.010421601136900228, 0.007579346281381514, 0.009947891994315498, 0.005684509711037428, 0.0061582188536221565, 0.004263382283278071, 0.009474182851728186, 0.008053055423968829, 0.010421601136900228, 0.010421601136900228, 0.0123164377072469, 0.008526764566556141, 0.009474182851728186, 0.013263855992418942, 0.016106110847937657, 0.010421601136900228, 0.014684983420178299, 0.012790146849834212, 0.012790146849834212, 0.01657981999052497, 0.010895310279487543, 0.019895783988630997, 0.017527238275697014, 0.020369493131215728, 0.024159166271909067, 0.027475130270012513, 0.036001894836568654, 0.04121269540502135, 0.06537186167693042, 0.07768829938417732, 0.10042633822832703, 0.12363808621506406, 0.15348176219801055, 0.18569398389388844, 0.23164377072477518, 0.2539081004263376, 0.30270014211274304, 0.34201800094741774, 0.3846518237801984, 0.42728564661297913, 0.47702510658455666, 0.5300805305542402, 0.577925153955471, 0.6205589767882517, 0.6674561819043104, 0.7171956418758879, 0.7550923732828033, 0.7981999052581713, 0.836570345807674, 0.8668877309332078, 0.8986262434864983, 0.9199431549028887, 0.941260066319279, 0.9597347228801507, 0.9711037423022255, 0.9748934154429189, 1.0, 0.9853150165798191, 0.9810516342965411, 0.9729985788725722, 0.9663666508763628, 0.9549976314542878, 0.9426811937470384, 0.9275225011842728, 0.9071530080530544, 0.9043107531975357, 0.8882046423495981, 0.8630980577925144, 0.8526764566556142, 0.8389388915206053, 0.8162008526764556, 0.796778777830412, 0.7759355755566089, 0.7588820464234967, 0.7385125532922783, 0.7261961155850314, 0.7096162955945039, 0.6920890573188069, 0.6721932733301759, 0.6537186167693042, 0.6385599242065361, 0.6191378493604923, 0.601136901942208, 0.5897678825201331, 0.5712932259592615, 0.5523448602558025, 0.5409758408337277, 0.5296068214116528, 0.5187115111321653, 0.501657981999053, 0.4860255802937001, 0.47702510658455666, 0.4642349597347224, 0.4519185220274755, 0.43770724774988196, 0.4258645191852198, 0.4135480814779729, 0.39696826148744535, 0.3912837517764079, 0.3784936049265737, 0.35765040265277065, 0.35480814779725195, 0.33538607295120826, 0.3306489815253429, 0.3197536712458553, 0.30743723353860586, 0.30743723353860586, 0.29559450497394624, 0.2913311226906682, 0.2861203221222155, 0.2700142112742778, 0.2719090478446219, 0.25580293699668427, 0.25248697299857825, 0.24538133585978145, 0.23827569872098467, 0.23211747986735992, 0.22595926101373776, 0.216485078162007, 0.2145902415916629, 0.2093794410232102, 0.2046423495973474, 0.19753671245855062, 0.19516816674561663, 0.19137849360492587, 0.18190431075319768, 0.1847465656087164, 0.17621980104216026, 0.16011369019422003, 0.16153481762197938, 0.1572714353387013, 0.15300805305542323, 0.1496920890573172, 0.14637612505921377, 0.14163903363334837, 0.14163903363334837, 0.13121743249644557, 0.1269540502131675, 0.12648034107058276, 0.12742775935575482, 0.1245855045002361, 0.12126954050213007, 0.12553292278540815, 0.11937470393178598, 0.12032212221695802, 0.11890099478919867, 0.11179535765040188, 0.11084793936522984, 0.10990052108005521, 0.10326859308384574, 0.10563713879677715, 0.1070582662245365, 0.10753197536712382, 0.09853150165798036, 0.09758408337280831, 0.09900521080056768, 0.09616295594504896};
