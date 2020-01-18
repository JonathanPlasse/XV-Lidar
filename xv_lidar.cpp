#include "xv_lidar.h"

// initialization
Lidar::Lidar() : rpmPID(xv_config.Kp, xv_config.Ki, xv_config.Kd), aryDist{}, aryQuality{0} {
  pwm_val = 127;
  motor_check_timer = millis();
  motor_check_interval = 200;
  rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
  rpm_err = 0;
  
  inByte = 0;  // incoming serial byte
  motor_rph_high_byte = 0;
  motor_rph_low_byte = 0;
  motor_rph = 0;
  startingAngle = 0;



  initConfig();
  pinMode(xv_config.motor_pwm_pin, OUTPUT);
  Serial.begin(115200);                    // USB serial
  Serial1.begin(115200);                   // XV LDS data


  rpmPID.setOutputLimits(xv_config.pwm_min, xv_config.pwm_max);
  rpmPID.setTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  rpmPID.setMode(true);

  pinMode(ledPin, OUTPUT);

  eState = eState_Find_COMMAND;
  for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // Initialize
    Packet[ixPacket] = 0;
  ixPacket = 0;

  curMillis = oldMillis = 0;
  while(!Serial);
}

void Lidar::run() {
  byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    if (xv_config.raw_data)
      Serial.write(inByte);                 // relay

    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          processSpeed();                             // process the speed
          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }
          for (int ix = 0; ix < N_DATA_QUADS; ix++) {
            if (xv_config.aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
              scan_data.angle = startingAngle + ix;
              scan_data.distance = aryDist[ix];
              scan_data.quality = aryQuality[ix];
              writeData(&scan_data, sizeof(scan_data));
            }  // if (xv_config.aryAngles[startingAngle + ix])
          }  // for (int ix = 0; ix < N_DATA_QUADS; ix++)
        }  // if (eValidatePacket() == 0
        // initialize a bunch of stuff before we switch back to State 1
        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          aryDist[ix] = 0;
          aryQuality[ix] = 0;
          aryInvalidDataFlag[ix] = 0;
        }
        for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
          Packet[ixPacket] = 0;
        ixPacket = 0;
        eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte
      }  // if (ixPacket == PACKET_LENGTH)
    }  // if (eState == eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  if (xv_config.motor_enable) {
    curMillis = millis();
    if (curMillis - oldMillis > xv_config.sample_time) {
      rpmPID.setInput(motor_rpm);
      rpmPID.setSetpoint(xv_config.rpm_setpoint);
      rpmPID.compute();
      pwm_val = rpmPID.getOutput();

      if (pwm_val != pwm_last) {
        analogWrite(xv_config.motor_pwm_pin, pwm_val);  // replacement for analogWrite()
        pwm_last = pwm_val;
      }
    }
    motorCheck();
  }  // if (xv_config.motor_enable)
}




/*
   processIndex - Process the packet element 'index'
   index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
      (packet 89, readings 356 to 359).
   Enter with: N/A
   Uses:       Packet
               ledState gets toggled if angle = 0
               ledPin = which pin the LED is connected to
               ledState = LED on or off
               xv_config.show_dist = true if we're supposed to show distance
               curMillis = milliseconds, now
               lastMillis = milliseconds, last time through this subroutine
               xv_config.show_interval = true ==> display time interval once per revolution, at angle 0
   Calls:      digitalWrite() - used to toggle LED pin
   Returns:    The first angle (of 4) in the current 'index' group
*/
uint16_t Lidar::processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);
  } // if (angle == 0)
  return angle;
}

/*
   processSpeed- Process the packet element 'speed'
   speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value
      in RPM represented in fixed point, with 6 bits used for the decimal part).
   Enter with: N/A
   Uses:       Packet
               angle = if 0 then enable display of RPM and PWM
               xv_config.show_rpm = true if we're supposed to display RPM and PWM
*/
void Lidar::processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}

/*
   Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
     byte 0 : <distance 7:0>
     byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
     byte 2 : <signal strength 7:0>
     byte 3 : <signal strength 15:8>
*/
/*
   processDistance- Process the packet element 'distance'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
                                       so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
   Calls:      N/A
   Exits with: 0 = okay
   Error:      1 << 7 = INVALID_DATA_FLAG is set
               1 << 6 = STRENGTH_WARNING_FLAG is set
*/
byte Lidar::processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}

/*
   processSignalStrength- Process the packet element 'signal strength'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               quality[] = signal quality
   Calls:      N/A
*/
void Lidar::processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

/*
   eValidatePacket - Validate 'Packet'
   Enter with: 'Packet' is ready to check
   Uses:       CalcCRC
   Exits with: 0 = Packet is okay
   Error:      non-zero = Packet is no good
*/
byte Lidar::eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b))
    return VALID_PACKET;                       // okay
  else
    return INVALID_PACKET;                     // non-zero = bad CRC
}

/*
   initConfig
*/
void Lidar::initConfig() {
  xv_config.motor_pwm_pin = 23;  // pin connected N-Channel Mosfet
  xv_config.rpm_setpoint = 300;  // desired RPM
  xv_config.rpm_min = 200;
  xv_config.rpm_max = 300;
  xv_config.pwm_min = 25;
  xv_config.pwm_max = 255;
  xv_config.sample_time = 20;
  xv_config.Kp = 0.5;
  xv_config.Ki = 0.25;
  xv_config.Kd = 0.0;

  xv_config.motor_enable = true;
  xv_config.raw_data = false;
  for (int ix = 0; ix < N_ANGLES; ix++)
    xv_config.aryAngles[ix] = true;
}

/*
   doSetAngle - Multi-angle range(s) implementation - DSH
   Command: SetAngles ddd, ddd-ddd, etc.
   Enter with: N/A
   Uses:       xv_config.aryAngles (an array of 360 booleans) is set to appropriate values
   Calls:      showDist
   Exits with: N/A
   TEST THIS STRING:  SetAngles 16-20, 300-305, 123-124, 10
*/
void Lidar::setAngle() {
  /* for (int ix = 0; ix < N_ANGLES; ix++)
    xv_config.aryAngles[ix] = true; */
}

void Lidar::motorOff() {
  xv_config.motor_enable = false;
  analogWrite(xv_config.motor_pwm_pin, 0);
}

void Lidar::motorOn() {
  xv_config.motor_enable = true;
  analogWrite(xv_config.motor_pwm_pin, pwm_val);
  rpm_err = 0;  // reset rpm error
}

void Lidar::motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
    if ((motor_rpm < xv_config.rpm_min or motor_rpm > xv_config.rpm_max) and pwm_val > 250) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
      motorOff();
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    motor_check_timer = millis();
  }
}
