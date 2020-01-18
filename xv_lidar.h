#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <pid.h>
#include <binserial.h>

const int N_ANGLES = 360;                // # of angles (0..359)
const int SHOW_ALL_ANGLES = N_ANGLES;    // value means 'display all angle data, 0..359'

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int OFFSET_DATA_DISTANCE_LSB = 0;
const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"



/* REF: https://github.com/Xevel/NXV11/wiki
The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
interrupted by the supports of the cover) .
*/
const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
/*
The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
reflections (glass... ).
*/
const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet


const int ledPin = 13;

class Lidar {
public:
    Lidar(); // Initialize the Lidar class
    void run(); // run in the loop
    uint16_t processIndex(); // Process the index of the packet
    void processSpeed(); // Process the speeds of the packet
    byte processDistance(int iQuad); // Process the distances of the packet
    void processSignalStrength(int iQuad); // Process the Signal Strengths of the packet
    byte eValidatePacket(); // Check the validity of the packet
    void initConfig(); // Init the lidar config
    void setAngle();
    void motorOff();
    void motorOn();
    void motorCheck();
    void setRPM();
    void setKp();
    void setKi();
    void setKd();
    void setSampleTime();

private:
    float pwm_val = 127;          // start with ~50% power
    float pwm_last;
    float motor_rpm;
    unsigned long now;
    unsigned long motor_check_timer = millis();
    unsigned long motor_check_interval = 200;
    unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
    unsigned int rpm_err = 0;

    int Packet[PACKET_LENGTH];                 // an input packet
    int ixPacket = 0;                          // index into 'Packet' array
    
    int eState = eState_Find_COMMAND;
    PID rpmPID;

    uint8_t inByte = 0;  // incoming serial byte
    uint8_t motor_rph_high_byte = 0;
    uint8_t motor_rph_low_byte = 0;
    uint16_t aryDist[N_DATA_QUADS] = {0, 0, 0, 0};   // there are (4) distances, one for each data quad
    // so the maximum distance is 16383 mm (0x3FFF)
    uint16_t aryQuality[N_DATA_QUADS] = {0, 0, 0, 0}; // same with 'quality'
    uint16_t motor_rph = 0;
    uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

    boolean ledState = LOW;

    struct config_t {
        int motor_pwm_pin;            // pin connected to mosfet for motor speed control
        float rpm_setpoint;          // desired RPM (uses float to be compatible with PID library)
        float rpm_min;
        float rpm_max;
        float pwm_max;              // max analog value.  probably never needs to change from 1023
        float pwm_min;              // min analog pulse value to spin the motor
        int sample_time;             // how often to calculate the PID values

        // PID tuning values
        float Kp;
        float Ki;
        float Kd;

        boolean motor_enable;        // to spin the laser or not.  No data when not spinning
        boolean raw_data;            // to retransmit the seiral data to the USB port
        boolean aryAngles[N_ANGLES]; // array of angles to display
    }
    xv_config;

    struct scan_t {
        uint16_t angle;
        uint16_t distance;
        uint16_t quality;
    }
    scan_data;

    uint32_t curMillis, oldMillis;
};

#endif