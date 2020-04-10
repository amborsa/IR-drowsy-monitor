#ifndef SI1153_h
#define SI1153_h

#define IR_ADDRESS 0x53 // defining Si115x address

// register addresses
#define PART_ID 0x00
#define REV_ID 0x01
#define MFR_ID 0x02
#define INFO0 0x03
#define INFO1 0x04
#define HOSTIN3 0x07
#define HOSTIN2 0x08
#define HOSTIN1 0x09
#define HOSTIN0 0x0A
#define COMMAND 0x0B
#define IRQ_ENABLE 0x0F
#define RESPONSE1 0x10
#define RESPONSE0 0x11
#define IRQ_STATUS 0x12
#define HOSTOUT0 0x13
#define HOSTOUT1 0x14
#define HOSTOUT2 0x15
#define HOSTOUT3 0x16
#define HOSTOUT4 0x17
#define HOSTOUT5 0x18
#define HOSTOUT6 0x19
#define HOSTOUT7 0x1A
#define HOSTOUT8 0x1B
#define HOSTOUT9 0x1C
#define HOSTOUT10 0x1D
#define HOSTOUT11 0x1E
#define HOSTOUT12 0x1F
#define HOSTOUT13 0x20
#define HOSTOUT14 0x21
#define HOSTOUT15 0x22
#define HOSTOUT16 0x23
#define HOSTOUT17 0x24
#define HOSTOUT18 0x25
#define HOSTOUT19 0x26
#define HOSTOUT20 0x27
#define HOSTOUT21 0x28
#define HOSTOUT22 0x29
#define HOSTOUT23 0x2A
#define HOSTOUT24 0x2B
#define HOSTOUT25 0x2C

// parameter table addresses
#define I2C_ADDR 0x00
#define CHAN_LIST 0x01
#define ADCCONFIG0 0x02
#define ADCSENS0 0x03
#define ADCPOST0 0x04
#define MEASCONFIG0 0x05
#define ADCCONFIG1 0x06
#define ADCSENS1 0x07
#define ADCPOST1 0x08
#define MEASCONFIG1 0x09
#define ADCCONFIG2 0x0A
#define ADCSENS2 0x0B
#define ADCPOST2 0x0C
#define MEASCONFIG2 0x0D
#define ADCCONFIG3 0x0E
#define ADCSENS3 0x0F
#define ADCPOST3 0x10
#define MEASCONFIG3 0x11
#define ADCCONFIG4 0x12
#define ADCSENS4 0x13
#define ADCPOST4 0x14
#define MEASCONFIG4 0x15
#define ADCCONFIG5 0x16
#define ADCSENS5 0x17
#define ADCPOST5 0x18
#define MEASCONFIG5 0x19
#define MEASRATE_H 0x1A
#define MEASRATE_L 0x1B
#define MEASCOUNT0 0x1C
#define MEASCOUNT1 0x1D
#define MEASCOUNT2 0x1E
#define LED1_A 0x1F
#define LED1_B 0x20
#define LED3_A 0x21
#define LED3_B 0x22
#define LED2_A 0x23
#define LED2_B 0x24
#define THRESHOLD0_H 0x25
#define THRESHOLD0_L 0x26
#define THRESHOLD1_H 0x27
#define THRESHOLD1_L 0x28
#define UPPER_THRESHOLD_H 0x29
#define UPPER_THRESHOLD_L 0x2A
#define BURST 0x2B
#define LOWER_THRESHOLD_H 0x2C
#define LOWER_THRESHOLD_L 0x2D

// commands
#define RESET_CMD_CTR 0x00
#define RESET_SW 0x01
#define FORCE 0x11
#define PAUSE 0x12
#define START 0x13
#define PARAM_QUERY 0x40 // must be bit-wise ORed with parameter table address to be queried
	// NOTE: this writes INPUT0 into the parameter byte, and sets RESPONSE1 to be INPUT0 too
#define PARAM_SET 0x80 // must be bit-wise ORed with parameter table address to be set
	// NOTE: this writes the parameter byte into RESPONSE1

#endif
