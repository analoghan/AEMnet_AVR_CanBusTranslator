#include <SPI.h>
#include <mcp_can.h>

// <--- Begin Display Math for Gauges
int i = 0;
float rawval = 0; // Setup raw sensor value
float kpaval = 0; // Setup kPa value
int8_t boost = 0; // Setup boost value
float barboost = 0; // Setup value for boost bar
float vac = 0; // Setup vacuum value
float peak = 0; // Setup peak value
float lambda1_raw = 0 ; // Raw Lambda Sensor Value 1
float lambda1 = 0 ; // Lambda
int8_t afr1 = 109 ; // AFR
float lambda2_raw = 0 ; // Raw Lambda Sensor Value 2
float lambda2 = 0 ; // Lambda
int8_t afr2 = 99 ; // AFR
float e85_raw = 0 ; // E85 Raw
int8_t e85 = 65 ; // E85 Percentage
int8_t rawboosttgt = 0 ; // Setup Raw Boost Target Input

// End Display Math for Gauges --->

// <--- Begin CAN Bus Definitions

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 2                         // Set INT to pin 2
MCP_CAN CAN0(10);                           // Set CS to pin 10

// End CAN Bus Definitions --->

// <--- Begin AEMNet Calculations per AEM CAN PDF Guide

const float RPM_SCALE = .39063;
const float ENG_LOAD_SCALE = .0015259;
const float ENG_THROTTLE_SCALE = .0015259;
const float ANALOG_SCALE = .00007782;
const float O2_SCALE = .00390625;
const float SPEED_SCALE = .00390625;
const float IGN_SCALE = .35156;
const float BATT_VOLTAGE_SCALE = .0002455;
const float TPS_SCALE = .0015259;
const float MAP_SCALE = .014504;
const float FUELPRESS_SCALE = .580151;
const float OILPRESS_SCALE = .580151;
const float LAMBDATARGET_SCALE = .00390625;
const float INJDUTY_SCALE = .392157;
const float FLEXFUEL_SCALE = .392157;
const float SPARKCUT_SCALE = .39063;
const float FUELCUT_SCALE = .39063;

int8_t coolantC = 100;
int8_t iatC = 35;
int8_t oilC = 90;

int MAP = 33;
int VE;
int fuelpress;
int oilpress;
int lambdatgt;
int injpw;
int injdutycycle;
int flex;
int sparkcut;
int fuelcut;
int oilF;
int rpm;
int load;
int tps;
int coolantF;
int iatF;
int o2_a; // O2 #1 Lambda
int o2_b; // O2 #2 Lambda
int vehicleSpeed; // Vehicle Speed
int timing; // Timing
byte gear; // Gears
int volts; // Voltage
int afr_target; // AFR Target
int boosttgt; // Boost Target
int can4byte6;
int can4byte7;
int can5byte7;
int launchRampTime;
int clutchPress;
int injLambdaFB;
int modeSw;
int waterPress;
int crankPress;
int estTrq;
int injProbability;
int sparkProbability;
int lambdaTrimKnock;
int baroPress;
int launchTimerArmed;
int loggingActive;
int modeSelectIgn;
int modeSelectLambda;
int modeSelectDBW;
int VTEC;
int transTemp;
int sparkCutRPM;
int fuelCutRPM;
int errorThrottle;
int errorCoolantTemp;
int errorFuelPress;
int errorOilPress;
int errorEBP;
int errorMAP;
int errorAirTemp;
int errorBaro;
int vvc1aTiming;
int vvc2aTiming;
int vvc1bTiming;
int vvc2bTiming;
int vvc1Target;
int vvc2Target;
int bootControl;
int bootFBPID;
int tcFuelCut;
int tcSparkCut;
int tcRetard;
int tcTqReduceDBW;
int threeStepTgtFuel;
int threeSteptgtSpark;
int threeStepFuel;
int threeStepSpark;
int threeStepSw;
int dlWheelSpeed;
int drWheelSpeed;
int nlWheelSpeed;
int nrWheelSpeed;
int knockFBCyl1;
int knockFBCyl2;
int knockFBCyl3;
int knockFBCyl4;
int knockFBCyl5;
int knockFBCyl6;
int knockFBCyl7;
int knockFBCyl8;

// End AEMNet Calculations per AEM CAN PDF Guide --->

void setup()
{
  Serial.begin(256000);                        
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s in STDEXT mode to allow filters.
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.init_Mask(0, 1, 0x1FFFFFF0);                // Set CAN filter mask 0
  CAN0.init_Mask(1, 1, 0x1FFFFFF0);                // Set CAN filter mask 1

  CAN0.init_Filt(0, 1, 0x01F0A000);                // Filter for CAN IDs 0-F

  CAN0.setMode(MCP_NORMAL);                        // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                        // Configuring pin for /INT input

//  attachInterrupt(CAN0_INT, canbus_read, CHANGE); // Interrupt for CAN bus data receipt
}

// <--- Begin CAN Bus function

void loop()
{
  if (!digitalRead(CAN0_INT))                        // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    unsigned char canId = CAN0.getCanId();

    if (canId == 0) {
      uint16_t rawRPM = (uint16_t)rxBuf[0] << 8;
      rawRPM |= rxBuf[1];
      rpm = rawRPM * RPM_SCALE;

      uint16_t rawLoad = (uint16_t)rxBuf[2] << 8;
      rawLoad |= rxBuf[3];
      load = rawLoad * ENG_LOAD_SCALE;

      uint16_t rawTPS = (uint16_t)rxBuf[4] << 8;
      rawTPS |= rxBuf[5];
      tps = rawTPS * TPS_SCALE;

      iatC = rxBuf[6];

      iatF = ((double)iatC * 1.8) + 32;

      coolantC = rxBuf[7];

      coolantF = ((double)coolantC * 1.8) + 32;
    }

    else if (canId == 3) {
      uint8_t rawo2_a = (uint8_t)rxBuf[0];
      o2_a = rawo2_a * O2_SCALE + 0.5;
      afr1 = (o2_a * 14.7); // Calculate AFR

      uint8_t rawo2_b = (uint8_t)rxBuf[1];
      o2_b = rawo2_b * O2_SCALE + 0.5;
      afr2 = (o2_b * 14.7); // Calculate AFR

      uint16_t rawSpeed = (uint16_t)rxBuf[2] << 8;
      rawSpeed |= rxBuf[3];
      vehicleSpeed = rawSpeed * SPEED_SCALE;

      gear = rxBuf[4];

      uint8_t rawign = (uint8_t)rxBuf[5];
      timing = rawign * IGN_SCALE - 17;

      uint16_t rawVolts = (uint16_t)rxBuf[6] << 8;
      rawVolts |= rxBuf[7];
      volts = rawVolts * BATT_VOLTAGE_SCALE;
    }

    else if (canId == 4) {
      uint16_t rawMAP = (uint16_t)rxBuf[0] << 8;
      rawMAP |= rxBuf[1];
      MAP = rawMAP * MAP_SCALE;

      uint8_t rawVE = (uint8_t)rxBuf[2];
      VE = rawVE;

      uint8_t rawfuelpress = (uint8_t)rxBuf[3];
      fuelpress = rawfuelpress * FUELPRESS_SCALE;

      uint8_t rawoilpress = (uint8_t)rxBuf[4];
      oilpress = rawoilpress * OILPRESS_SCALE;

      uint8_t rawlambda = (uint8_t)rxBuf[5];
      lambdatgt = rawlambda * LAMBDATARGET_SCALE + 0.5;
      afr_target = (lambdatgt * 14.7); // Calculate AFR  

      can4byte6 = rxBuf[6];
      can4byte7 = rxBuf[7];
                     
    }

    else if (canId == 5) {
      uint16_t rawLaunchRampTime = (uint16_t)rxBuf[0] << 8;
      rawLaunchRampTime |= rxBuf[1];
      launchRampTime = rawLaunchRampTime / 10;

      clutchPress = (rxBuf[6] * 5);

      can5byte7 = rxBuf[7];
      
    }

    else if (canId == 6) {
      uint8_t rawinjpw = (uint8_t)rxBuf[0];
      injpw = rawinjpw * 0.1;

      injLambdaFB = ((rxBuf[1] * 0.5) - 64);

      uint8_t rawinjdc = (uint8_t)rxBuf[2];
      injdutycycle = rawinjdc * INJDUTY_SCALE;

      modeSw = rxBuf[3];
      waterPress = (rxBuf[4] * .580151);
      crankPress = rxBuf[5];

      uint16_t rawEstTrq = (uint16_t)rxBuf[6] << 8;
      rawEstTrq |= rxBuf[7];
      estTrq = ((rawEstTrq * .1) - 3276.8);
    }

    else if (canId == 7) {
      uint8_t rawflex = (uint8_t)rxBuf[4];
      flex = rawflex * FLEXFUEL_SCALE;

      uint8_t rawoilC = (uint8_t)rxBuf[6];
      oilC = rawoilC - 50;

      oilF = ((double)oilC * 1.8) + 32;
    }

    else if (canId == 8) {
      uint16_t rawsparkcut = (uint16_t)rxBuf[1] << 8;
      rawsparkcut |= rxBuf[2];
      sparkcut = rawsparkcut * SPARKCUT_SCALE;

      uint16_t rawfuelcut = (uint16_t)rxBuf[3] << 8;
      rawfuelcut |= rxBuf[4];
      fuelcut = rawfuelcut * FUELCUT_SCALE;
    }

    else if (canId == 10) {
      uint16_t rawboosttgt = (uint16_t)rxBuf[1] << 8;
      rawboosttgt |= rxBuf[2];
      boosttgt = rawboosttgt * 0.01;

      //            Serial.print("Boost Target: ");
      //            Serial.print(boosttgt);
      //            Serial.println();
    }

  }

  SendCANFramesToSerial();
  
}

void SendCANFramesToSerial()
{
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file

  // build 1st CAN frame, RPM, MAP, CLT, TPS
  buf[0] = ((rpm >> 8) & 0xff);
  buf[1] = (rpm & 0xff);
  buf[2] = ((load >> 8) & 0xff);
  buf[3] = (load & 0xff);
  buf[4] = ((tps >> 8) & 0xff);
  buf[5] = (tps & 0xff);  
  buf[6] = iatC;
  buf[7] = coolantC;

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, buf);

  // build 2nd CAN frame, AFR1, AFR2, Gear, Timing, Voltage
  buf[0] = afr1;
  buf[1] = afr2;
  buf[2] = ((vehicleSpeed >> 8) & 0xff);
  buf[3] = (vehicleSpeed & 0xff);
  buf[4] = gear;
  buf[5] = timing;
  buf[6] = ((volts) & 0xff);
  buf[7] = (volts & 0xff);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, buf);

  // build 3rd CAN frame, MAP, VE, Fuel Pressure, Oil Pressure, AFR Target, Status Bits
  buf[0] = ((MAP >> 8) & 0xff);
  buf[1] = (MAP & 0xff);  
  buf[2] = VE;
  buf[3] = fuelpress;
  buf[4] = oilpress;
  buf[5] = afr_target;
  buf[6] = can4byte6;
  buf[7] = can4byte7;

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, buf);

  // build 4th frame, Launch Ramp Time, Clutch Prssure, Switch Status
  buf[0] = ((launchRampTime >> 8) & 0xff);
  buf[1] = (launchRampTime & 0xff);
  buf[2] = (0xff);
  buf[3] = (0xff);
  buf[4] = (0xff);
  buf[5] = (0xff);
  buf[6] = clutchPress;
  buf[7] = can5byte7;

  // write 4th CAN frame to serial
  SendCANFrameToSerial(3203, buf);

  // build 5th frame, Oil Pressure, Spark Cut, Fuel Cut, Boost Target
  buf[0] = ((oilpress >> 8) & 0xff);
  buf[1] = (oilpress & 0xff);
  buf[2] = ((sparkcut >> 8) & 0xff);
  buf[3] = (sparkcut & 0xff);
  buf[4] = ((fuelcut >> 8) & 0xff);
  buf[5] = (fuelcut & 0xff);
  buf[6] = ((boosttgt >> 8) & 0xff);
  buf[7] = (boosttgt & 0xff);

  // write 5th CAN frame to serial
  SendCANFrameToSerial(3205, buf);
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const unsigned long serialBlockTag = 0x11223344;
  Serial.write((const byte*)&serialBlockTag, 4);

  // the CAN frame id number
  Serial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}
