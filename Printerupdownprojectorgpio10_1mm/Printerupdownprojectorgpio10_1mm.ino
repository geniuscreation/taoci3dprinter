
//  includes
#include <b9SoftwareSerial.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <PinChangeInt.h>
#include <TimerOne.h>

// defines

// EEPROM (used for storing calibrated Z home offset
#define EE_SCHEMA 112
#define EE_ADDRESS_SCHEMA 0 
#define EE_ADDRESS_Z_HOME 2
#define EE_ADDRESS_NATIVEX 4
#define EE_ADDRESS_NATIVEY 6
#define EE_ADDRESS_XYPSIZE 8
#define EE_ADDRESS_HALFLIFE 10

// Optical sensor inputs
#define Z_HOME    A1

// Manual Toggle switch inputs
#define B_UP      A2
#define B_DOWN    A3
#define B_LEFT    A4
#define B_RIGHT   A5


// Slider Motor Outputs
#define SLIDE_MTR_DIR 100//2
#define SLIDE_MTR_PWM 101//3
#define LEDTOPON 2

// Stepper Motor Outputs
#define Z_ENABLE 9
#define Z_MS1    8
#define Z_MS2    7
#define Z_MS3    6
#define Z_STEP   5
#define Z_DIR    4

// Projector's RS-232 Port I/O pins
#define PROJECTOR_RX 12
#define PROJECTOR_TX 13

// Projector control gpio Output
#define PROJECTOR_ON 10 // pin 10 is the projector swith
#define LED_ON 11


// Stepper Mode
#define Z_STEP_FULL 0
#define Z_STEP_HALF 1
#define Z_STEP_QTR  2
#define Z_STEP_8TH  3
#define Z_STEP_16TH 4

// Stepper Direction
#define Z_STEPUP  HIGH //low
#define Z_STEPDN  LOW  //high

// Stepper speeds in RPM
#define Z_MAXSPEED  120
#define Z_MINSPEED   60
#define Z_NORMALSPD 120
#define Z_RESETSPD  120

// Slide Speeds, 255 = full power
#define X_ManualSPEED  111

// Print Cycle states
#define CYCLE_OFF     0
#define CYCLE_OPEN    1
#define CYCLE_DOWN    2
#define CYCLE_SETTLE  3
#define CYCLE_EXPOSE  4
#define CYCLE_RELEASE 5
#define CYCLE_UP      6
#define CYCLE_BREATHE 7
#define CYCLE_FINISH  8

// Maximum time spent in print cycle without
// a command before we shut off projector as a saftey precaution
#define MAX_EXPOSURE_WATCHDOG 240000 // 240 seconds


// Z Movement states
#define Z_MOVING    0
#define Z_ARRIVED   1
#define Z_BROADCAST 2



// Motion state variables
int iZStatus     = Z_ARRIVED;

bool bReset = false;

// Persistent Calibration and setup settings
#define XNATIVEDEFAULT 1920
#define YNATIVEDEFAULT 1080
#define XYPSIZEDEFAULT 50
#define HALFLIFEDEFAULT 2000 // Hours before projector lamp output is reduced to 1/2 original value

int iNativeX = 0;
int iNativeY = 0;
int iXYPixelSize = 0;
int iHalfLife = 2000;

// Z Axis Position & Movement variables
#define ZHOMEOFFSETDEFAULT 4000
int iRefZeroOffset  = -1;       // Load persistant value from EEPROM during setup
int iUpperZLimit    = 20000;
int iLowerZLimit    = -500;
int iCurPos         = 0;
int iLastCurPos     = 0;
int iTargetPos      = 0;
int iZDriveSpeedCmd = Z_NORMALSPD;
int lastStepPulse   = LOW;

// Slider Position & Movement variables
unsigned long ulLastInteruptTime =  millis();


// Event Flags
boolean bFindZero = false;
boolean bEnableManual = true;

// Cycle variables
int iCycle = CYCLE_OFF;
int iLastCuredPos = 0;
int iNextCurePos = 0;


////////////////////////////////////////////////////////////////////////////////////
int hangbing = 0;
int ledtime=2000;
// Raise during print variables
int iReadyGap    = 0; // 100 = .025"
unsigned long ulBreatheStart = millis();
unsigned long ulBreatheDur  = 0;
unsigned long ulSettleStart = millis();
unsigned long ulSettleDur = 0;
int iRaiseSpeed  = Z_NORMALSPD;
int iLowerSpeed  = Z_NORMALSPD;

// Projector RS-232 interface & status variables
b9SoftwareSerial projectorSerial = b9SoftwareSerial(PROJECTOR_RX, PROJECTOR_TX);
unsigned long ulLastProjectorQueryTime = millis();
unsigned long ulLastProjectorMsgRcvdTime = millis();
bool bFirstProjectorQuery = true;
// We can only do one factory reset once the projector is turned on because it leaves us in an
// unknown menu state and we can not repeat the key stroke sequence again.
// So we only allow for one reset per power on command.
// This reset can be commanded manually with P7 or automaticaly with the B (base) command
bool bProjectorNeedsReset = false; 
bool bIsMirrored = false;
int iProjectorPwr = -1; // 0 = OFF, 1 = ON, -1 = ?
int iLampHours    = -1; // reported lamp hours, -1 = ?
unsigned long ulLastCmdReceivedTime = millis(); // reset every time we get a cmd from the host

// Broadcast variables
int iBC_Interval = 5000;  //millisecs between broadcasts, set to 0 to disable broadcasts
unsigned long ulLastTimedBroadcastTime = millis();
bool bVerbose = true; // Send comments to Serial stream

// Function Declarations
void BC_C(const __FlashStringHelper* s, String sVariable = "");
void BC_String(const __FlashStringHelper* s, String sVariable = "");


///////////////////////////////////////////////////////
//
// Program Setup
//
void setup() {

  // Set up the IO pins and interupts
  setupIO(); 
  
  // set up Serial library at 115200 bps
  Serial.begin(115200); 
  
  // set the data rate for the b9SoftwareSerial port
  projectorSerial.begin(9600);  
  ulLastCmdReceivedTime = millis(); // initilized
  ulLastProjectorMsgRcvdTime = millis();
  
  // set up stepper motor timer, iZDriveSpeedCmd is in RPM's
  // if iZDriveSpeedCmd =120 RPM then that's 2 Revs per second
  // The timer must fire twice per step (rise/fall) and there are 200 steps per rev
  // So for 120 RPM, thats 2 revs per sec, which is 400 steps per sec and each step requires the timer to fire twice
  // giving us a requirement to fire 800 time per second so the timer must fire every 0.00125 seconds which is once
  // every 1250 micro seconds (1 million micro seconds in a second).  1250 * 120 = 150,000.
  Timer1.initialize(150000/iZDriveSpeedCmd);
  // attach the motor intrupt service routine here
  Timer1.attachInterrupt( updateMotors ); 
  setZSpeed(iZDriveSpeedCmd);
  setZStepMode(Z_STEP_FULL);
  ZStepRelease();

   // Read EEPROM stored settings
  loadEEPromSettings();
 
  // Hello World....
  BC_V();
  bVerbose = false;
  BC_A();  // Projector control
  BC_I();  // Printer Units PU
  BC_J();  // Shutter available
  BC_K();  // Lamp Half Life
  BC_M();  // Max allowed Z position in PU
  BC_R();  // Reset Status
  BC_S(100);  // Current Slide Position
  BC_Y();  // Z Home Reference
  BC_Z();  // Current Z Position
  BC_D();  // Projector's X Resolution
  BC_E();  // Projector's Y Resolution
  BC_H();  // Calibrated XY Pixel Size in Microns  
}

///////////////////////////////////////////////////////
//
// Main Event Loop
//
void loop() {
  // Handle Manual input events
  if (bEnableManual==true)
  {
    // Control Z axis Stepper
    if (!digitalRead(B_DOWN)){   // Commanded to go lower, -Z
      iTargetPos = iCurPos - 1;
      iZStatus = Z_MOVING;
    }
    else if(!digitalRead(B_UP)){ // Commanded to higher, +Z
      iTargetPos = iCurPos + 1;
      iZStatus = Z_MOVING;
    }

  }
/////////////////////////////////////////////************************************************  
     if (!digitalRead(B_RIGHT)){ // Commanded to go right  

      hangbing=1;
    }
    else if(!digitalRead(B_LEFT)){ // Commanded to left

      hangbing=2;
    }
    else {
  
      hangbing=0;
    }
////////////////////////////////////////////////////******************************************  
  // Have we arrived at a target Z Position?
  if(iZStatus == Z_BROADCAST){
    iZStatus = Z_ARRIVED;
    BC_Z();
  }
  
   
  // Reset Home requested?
  if (bFindZero==true) {  // This is the main reset event
    bReset=false;
    bFindZero = false;
    setZero(false);
    //BC_S(0);
    bReset = true;
    iLastCuredPos = 0;
    iZStatus = Z_BROADCAST;
    BC_R();
  }
  
  // Handle Build Cycles
  HandleBuildCycles();

  // Any news from the Projector?
  if(iZStatus != Z_MOVING) // Only check if we're not busy with moving parts!
    HandleProjectorComm_Vivitek_D535();
  
  // Handle Time Based Broadcasts
  HandleTimedBroadcasts();
  
}


//////////////////////////////////////////////////////////////
//
// SerialEvent occurs whenever a new data comes in the
// hardware serial RX.  This routine is run each
// time loop() runs, so using delay inside loop can delay
// response.  Multiple bytes of data may be available.
//
void serialEvent() {
  int i, iDiff;
  char cInChar;
  int iNewPos;
  ulLastCmdReceivedTime = millis(); // set to time of serialEvent

  if(Serial.available()) {
    // get the new byte:
    cInChar = (char)Serial.read(); 
    switch (cInChar)  {
      
      case 'a':     // Request Acknowledgement
      case 'A':
        BC_C(F("Command:  Acknowledge")); 
        BC_V();  // Version & Model info
        BC_A();  // Projector control
        BC_J();  // Shutter available
        BC_I();  // Printer Units PU
        BC_K();  // Lamp Half Life
        BC_M();  // Max allowed Z position in PU
        BC_R();  // Reset Status
        BC_Y();  // Z Home Reference
        BC_S(100);  // Current Slide Position
        BC_Z();  // Current Z Position
        BC_D();  // Projector's X Resolution
        BC_E();  // Projector's Y Resolution
        BC_H();  // Calibrated XY Pixel Size in Microns
        break;     

      case 'b':     // Position to build Base layer
      case 'B':
        projectorReset();        
        bEnableManual = false;
        iLastCuredPos = 0;
        iNextCurePos = SerialReadInt();
        BC_C(F("Command: Cycle to initial Base Layer at: "),String(iNextCurePos)); 
        BC_S(100);
        iCycle = CYCLE_OPEN;      
        break;
      
      case 'c':      // Request current Status
      case 'C':
        BC_C(F("Command:  Request Current Status")); 
        BC_Z();
        BC_S(100);
        break;
      
      case 'd':     // Set Delay (Breathe) time, a pause before opening shutter
      case 'D':
        i = SerialReadInt();
        if (i>=0 && i<=60000) ulBreatheDur = i;
        BC_C(F("Command: Breathe Delay set to "), String(ulBreatheDur));
        break;     

      case 'e':     // Set "settle" Delay time, a pause before sending "Cycle Complete"
      case 'E':
        i = SerialReadInt();
        if (i>=0 && i<=60000) ulSettleDur = i;
        BC_C(F("Command: Settle Delay set to "), String(ulSettleDur));
        break;     

      case 'f':     // Position after building Final layer
      case 'F':
        bEnableManual = false;
        iLastCuredPos = iNextCurePos;
        iNextCurePos = SerialReadInt();
        BC_C(F("Command: Release and move to final position at: "),String(iNextCurePos));
       BC_S(0);
        iCycle = CYCLE_FINISH; 
        break;
        
      case 'g':     // Goto Z position
      case 'G':
        setZSpeed(iZDriveSpeedCmd);
        i = SerialReadInt();
        if(i==500)
        i=0;
        while(lastStepPulse==HIGH){delay(1);} // wait until step LOW      
        iTargetPos = i;
        
        if (iTargetPos>iUpperZLimit) iTargetPos = iUpperZLimit;
        else if (iTargetPos<iLowerZLimit) iTargetPos = iLowerZLimit;
        iZStatus = Z_MOVING;
        BC_C(F("Command: Goto Z position at: "),String(iTargetPos)); 
        break;     
      
      case 'h':        // Reset the projectors native X Resolution
      case 'H':
        i = SerialReadInt();
        if(i>=0)
        {
          iNativeX = i;
          storeRefNativeX();  // store new value in persistant EEPROM
        }
        BC_C(F("Command: Native X Resolution Set To "), String(iNativeX));
        break;     
         
      case 'i':        // Reset the projectors native Y Resolution
      case 'I':
        i = SerialReadInt();
        if(i>=0)
        {
          iNativeY = i;
          storeRefNativeY();  // store new value in persistant EEPROM
        }
        BC_C(F("Command: Native Y Resolution Set To "), String(iNativeY));
        break;     
         
      case 'j':     // Set Cycle Ready Gap
      case 'J':
        i = SerialReadInt();
        if (i>=0 && i<=1500) iReadyGap = i;
        BC_C(F("Command: Reposition Gap set to: "),String(iReadyGap)); 
        break;     
      
      case 'k':     // Set Print Cycle Raise Speed
      case 'K':
        i = SerialReadInt();
        if (i>=0 && i<=100)
        {
          float dPercent = (float)i/100.0;
          float dRange = Z_MAXSPEED - Z_MINSPEED;
          iRaiseSpeed = (dPercent * dRange) + Z_MINSPEED;          
          BC_C(F("Command: Percent Raise Speed Set To "), String(i));
          BC_C(F("Command: RPM Raise Speed Set To "), String(iRaiseSpeed));
        }
        else
        {
          BC_C(F("Command: Error, Percent Raise Speed Out of Range"));          
        }
        break;     

      case 'l':     // Set Print Cycle Lower Speed
      case 'L':
        i = SerialReadInt();
        if (i>=0 && i<=100) 
        {
          float dPercent = (float)i/100.0;
          float dRange = Z_MAXSPEED - Z_MINSPEED;
          iLowerSpeed = (dPercent * dRange) + Z_MINSPEED;          
          BC_C(F("Command: Percent Lower Speed Set To "), String(i));
          BC_C(F("Command: RPM Lower Speed Set To "), String(iLowerSpeed));
        }
        else
        {
          BC_C(F("Command: Error, Percent Lower Speed Out of Range"));          
        }
        break;     

      case 'm':     // Toggle Manual Contol Activation
      case 'M':
        i = SerialReadInt();
        if (i>0) 
        {  
          bEnableManual = true;
          BC_C(F("Command: Manual Controls Activated.")); 
        }
        else
        {  
          bEnableManual = false;
          BC_C(F("Command: Manual Controls Deactivated.")); 
        }
        break;           
      
      case 'n':     // Release and Position to build next Layer
      case 'N':
        bEnableManual = false;
        iDiff = iNextCurePos - iLastCuredPos; // The last increment, we should be similar this time
        iLastCuredPos = iNextCurePos;
        iNextCurePos = SerialReadInt();
        if(iNextCurePos < iLastCuredPos) { // This should never happen, but if it does reset iNextCurePos to use the last increment
          BC_C(F("ERROR:  Invalid Command: Release and cycle to Next Layer at: "),String(iNextCurePos)); 
          BC_C(F("NOTE:  Reset Next Cure Position to last cure position + last increment."));
          iNextCurePos = iLastCuredPos + iDiff;
        }
        BC_C(F("Command: Release and cycle to Next Layer at: "),String(iNextCurePos)); 
        BC_S(0);
       iCycle=CYCLE_RELEASE;
        break;     
      
      case 'o':     // Set the current position to xxxx, use with caution!
      case 'O':
        i = SerialReadInt();
        if(i>=-1000 && i<=iUpperZLimit){ // 1 to iUpperZLimit valid range
          BC_C(F("Command: Resetting current Z position to: "),String(i));
          iTargetPos = iCurPos = i;
          iNextCurePos = iLastCuredPos = i; //Reseting this incase we're in the middle of a print
          bReset = true;  // not realy, but we are tricking the system into believing it knows where it's at
          BC_Z();
        }
        else {
          BC_C(F("Error: Reset current Z position value out of limits.  Ignored."));           
        }
        break;     
      
      case 'p':     // Projector Power & Mode commands
      case 'P':
          i = SerialReadInt();
        if(i==0){ 
          BC_C(F("Command: Turning Projector OFF")); 
          digitalWrite(PROJECTOR_ON,LOW);
          digitalWrite(LED_ON,LOW);
          delay(1000);
        }
        else if(i==1){
          BC_C(F("Command: Turning Projector ON"));       
         digitalWrite(PROJECTOR_ON,HIGH);
          digitalWrite(LED_ON,HIGH);         
        }
        else if(i==12){
          projectorSerial.write("1 ena\n"); 
          projectorSerial.write("1 zero start\n");
          delay(1000);
          BC_C(F("Command: motor 1  zero start "));           
        }
        else if(i==13){
          projectorSerial.write("2 ena\n"); 
         projectorSerial.write("2 zero start\n");
          delay(1000); 
          BC_C(F("Command:motor 2 zero start "));
        }
       else if(i==14){
          projectorSerial.write("1 ena\n");
          delay(1000);
           projectorSerial.write("1 pos 110000\n");         
          BC_C(F("Command: motor 1 mov to 110000 "));
        }
        else if(i==15){
          projectorSerial.write("2 ena\n");
          delay(1000);
          projectorSerial.write("2 pos 15000\n");         
          BC_C(F("Command: motor 2 mov to 15000"));        
        }
        else if(i==16){
           projectorSerial.write("1 ena\n");
          delay(1000);
           projectorSerial.write("1 pos 0\n");
          BC_C(F("Command: motor 1 mov to 0 "));         
        }
        else if(i==17){
           projectorSerial.write("2 ena\n");
          delay(1000);
          projectorSerial.write("2 pos 0\n");          
          BC_C(F("Command: motor 2 mov to 0"));
        }
        else if(i==18) {
         digitalWrite(LEDTOPON,HIGH);
         delay(ledtime);
         digitalWrite(LEDTOPON,LOW); 
          BC_C(F("Command: TOPLED 16LIGHT ledtime s"));          
        }

        break;  

      case 'q':     // Set Broadcast interval in milliseconds, 0 = disabled
      case 'Q':
        i = SerialReadInt();
        if(i==0){
          BC_C(F("Command: Turning Broadcasts OFF")); 
          iBC_Interval = i;
        }
        else if(i>999 && i<60001){ // 1 to 60 sec valid range
          iBC_Interval = i;
          BC_C(F("Command: Broadcast Interval set to: "), String(iBC_Interval));
          ulLastTimedBroadcastTime = 0;
        }
        else {
          BC_C(F("Command: Broadcast Interval is: "), String(iBC_Interval));
        }
        break;  

      case 'r':     // Reset Home
      case 'R':
        i = SerialReadInt();
        if(i==99){
          BC_C(F("Command: Reset Factory Defaults")); 
          resetFactoryDefaults();          
        }
        else {
          BC_C(F("Command: Reset Home Positions")); 
          bFindZero = true;
          iCycle = CYCLE_OFF;
          bEnableManual = true;
        }
        break;
        
      case 's':     // Stop all motion, activate manual controls
      case 'S':
        BC_C(F("Command: STOP"));
        iCycle = CYCLE_OFF;
        bEnableManual = true;
        while(lastStepPulse==HIGH){delay(1);} // wait until step LOW      
        iTargetPos = iCurPos; 
        break;

      case 't':     // Toggle verbose Text Comments
      case 'T':
        i = SerialReadInt();
        if (i>0) {  
          bVerbose = true;
          BC_C(F("Command: Verbose Text Comments Activated.")); 
        } 
        else {  
          BC_C(F("Command: Verbose Text Comments Deactivated.")); 
          bVerbose = false;
        }
        break;     

      case 'u':        // Reset the projectors calibrated xy pixel size in microns
      case 'U':
        i = SerialReadInt();
        if(i>=0)
        {
          iXYPixelSize = i;
          storeRefXYPixelSize();  // store new value in persistant EEPROM
       }
        BC_C(F("Command: Calibrated XY Pixel Size Set To "), String(iXYPixelSize));
        break;     
         
      case 'v':      // Set Vat Position (0 - 100 %)
      case 'V':
       i=SerialReadInt();
        iNewPos=i*32;
        if(i==0)
        {
          BC_C(F("Command: Moving VAT to Closed Position:"),String(iNewPos));
          BC_S(0);
          break;
        }
        else if(i==100)
        {
          BC_C(F("Command: Moving VAT to Open Position:"),String(iNewPos));
          BC_S(100);
          break;
        }
        else 
        {
          BC_C(F("Command: Moving VAT to:"),String(iNewPos));
          BC_S(i);
        }
        break;  
        
      case 'w':     // Set Print Cycle Opening Speed
      case 'W':
        i = SerialReadInt();
        if (i>=0 && i<=100)
        {         
          BC_C(F("Command: Percent Open Speed Set To: "), String(i));
        }
        else
        {
          BC_C(F("Command: Error, Percent Open Speed Out of Range"));          
        }
        break;     

      case 'x':     // Set Print Cycle Closing Speed
      case 'X':
        i = SerialReadInt();
        if (i>=0 && i<=100)
        {        
          BC_C(F("Command: Percent Close Speed Set To: "), String(i));
        }
        else
        {
          BC_C(F("Command: Error, Percent Open Speed Out of Range"));          
        }
        break;     

      case 'y':        // Reset z Axis home reference
      case 'Y':
        i = SerialReadInt();
        iDiff = i - iRefZeroOffset;       
        if (i>0) {
          iCurPos+=iDiff;
          iTargetPos = iCurPos;
          iRefZeroOffset = i;
          storeRefZOffset();  // store new value in persistant EEPROM
        }
        BC_C(F("Command: Z Home Reference Set To "), String(iRefZeroOffset));
        break;     

      case 'z':        // Set Z Axis speed
      case 'Z':
        i = SerialReadInt();
        if (i>=Z_MINSPEED && i<=Z_MAXSPEED) iZDriveSpeedCmd = i;
        setZSpeed(iZDriveSpeedCmd);
        BC_C(F("Current z Drive Speed set to "), String(iZDriveSpeedCmd));
        break;  
        
      case '#':
        i = SerialReadInt();
        if (i>=0 && i<=60000) ledtime = i;   
        BC_C(F("Command: ledtop light  set to "), String(ledtime));
        break;      

      case '$':        // Reset Projector's Half Life value
        i = SerialReadInt();
        if (i>0) {
          iHalfLife = i;
          storeHalfLife();  // store new value in persistant EEPROM
        }
        BC_C(F("Command: Lamp Half Life Set to: "), String(iHalfLife));
        break;     

      default:
        break;  
    }
  }
}


///////////////////////////////////////////////////////
//
//  Handle Build Cycles
//
void HandleBuildCycles()
{
  switch (iCycle) 
  {
    case CYCLE_OPEN:
      if(1){
        // We've reached the open position
        BC_C(F("Shutter Open")); 
        if(iCurPos!=iNextCurePos){ // we need to lower down to the next cure position
          BC_C(F("Lowering Platform to next layer position.")); 
          iTargetPos = iNextCurePos;
          setZSpeed(iLowerSpeed);
          iZStatus = Z_MOVING;
          
        }
        iCycle = CYCLE_DOWN;
      }
      break;

    case CYCLE_DOWN:  
      if(iCurPos == iTargetPos){
        // We've reached the Lowered position
        BC_C(F("Lowered.  Pausing for settle...")); 
         projectorSerial.write("2 ena\n");
        projectorSerial.write("2 pos 0\n");//pingtai shangsheng
        delay(4000);
        ulSettleStart = millis();
        digitalWrite(Z_ENABLE, HIGH);
        iCycle = CYCLE_SETTLE;
      }
      break;
      
    case CYCLE_SETTLE:  
      if(millis()-ulSettleStart > ulSettleDur){
        // We've reached the end of our settling duration
        BC_C(F("Settle Pause Finished.  Ready to Expose.")); 
        BC_F();
        digitalWrite(Z_ENABLE, HIGH);
        digitalWrite(LED_ON, HIGH); 
        iCycle = CYCLE_EXPOSE;
      }
    break;

    case CYCLE_RELEASE:  
      if(1){
        // We've reached the closed position
        BC_C(F("Released, Shutter Closed")); 
        BC_C(F("Raising Platform to next layer position + clearance Gap.")); 
         digitalWrite(LED_ON, LOW); 
         delay(1000);
         digitalWrite(Z_ENABLE, HIGH);
          projectorSerial.write("2 ena\n");
         delay(500);
        projectorSerial.write("2 pos 15000\n");//pingtaixiajiang
         delay(4000); 
        iTargetPos = iNextCurePos+iReadyGap;  // Never lower with a part in process!
       
        if(iTargetPos < 0) iTargetPos = 0;
        setZSpeed(iRaiseSpeed);
        iZStatus = Z_MOVING;
        iCycle = CYCLE_UP;
        digitalWrite(Z_ENABLE, HIGH);
      }
      break;

    case CYCLE_UP:  
      if(iCurPos == iTargetPos){
        // We've reached the Raised position
        BC_C(F("Raised.  Pausing for breathe...")); 
         projectorSerial.write("1 pos 110000\n");//guabanhouyidao100000
       delay(12000);
        projectorSerial.write("1 pos 0\n");//guabanguilingwei
        delay(12000);//guabanhouyidong
       digitalWrite(Z_ENABLE, HIGH);
        ulBreatheStart = millis();
        iCycle = CYCLE_BREATHE;
      }
      break;

    case CYCLE_BREATHE:  
      if(millis() - ulBreatheStart > ulBreatheDur){
        // We've reached the end of our breathing duration
        BC_C(F("Breathe Pause Finished.  Opening Shutter."));
       BC_S(100); 
        digitalWrite(Z_ENABLE, HIGH); 
        iCycle = CYCLE_OPEN;
      }
      break;

    case CYCLE_FINISH:  
      if(1){
        // We've reached the closed position
        BC_C(F("Released"));
        BC_C(F("Raising Platform to final position.")); 
        digitalWrite(LED_ON,LOW);
        delay(1000);
        projectorSerial.write("2 pos 15000\n");//pingtaixiajiang
         delay(4000); 
        if(iNextCurePos>iCurPos)
          iTargetPos = iNextCurePos;  // Never lower with a part in process!
        setZSpeed(iRaiseSpeed);
        iZStatus = Z_MOVING;
        iCycle = CYCLE_OFF;
      }
      break;

    case CYCLE_EXPOSE:
      break;    

    case CYCLE_OFF:
      if(iCurPos == iTargetPos && bEnableManual==false ){
        // We've reached the Raised position, last step of the F command
        bEnableManual = true;
        setZSpeed(iZDriveSpeedCmd);
        BC_F();
       }
      break;
      
    default:
      break;    
  }
}

///////////////////////////////////////////////////////
//
// Initialize the IO parameters
void setupIO()
{
  // Projector RS-232 serial I/O
  pinMode(PROJECTOR_RX, INPUT);
  PCintPort::attachInterrupt(PROJECTOR_RX, &projectorRxChange, CHANGE);
  pinMode(PROJECTOR_TX, OUTPUT); 
  
  pinMode(B_DOWN, INPUT);       // DOWN
  digitalWrite(B_DOWN, HIGH);   // Set pull up resistor
  pinMode(B_UP, INPUT);         // UP
  digitalWrite(B_UP, HIGH);     // Set pull up resistor
  pinMode(B_LEFT, INPUT);       // SLIDE LEFT
  digitalWrite(B_LEFT, HIGH);   // Set pull up resistor
  pinMode(B_RIGHT, INPUT);      // Slide RIGHT
  digitalWrite(B_RIGHT, HIGH);  // Set pull up resistor

  pinMode(Z_HOME, INPUT);       // Z = zero sensor
  digitalWrite(Z_HOME, HIGH);   // Set pull up resistor
  
  pinMode(PROJECTOR_ON,OUTPUT);
  digitalWrite(PROJECTOR_ON,LOW);//turn off projector
  pinMode(LED_ON,OUTPUT);
  digitalWrite(LED_ON,LOW);//TURN OFF LED
  pinMode(LEDTOPON,OUTPUT);
  digitalWrite(LEDTOPON,LOW);//trun off ledtopon
  
  
  pinMode(SLIDE_MTR_PWM, OUTPUT);
  pinMode(SLIDE_MTR_DIR, OUTPUT);

  pinMode(Z_ENABLE, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  //pinMode(Z_MS1, OUTPUT);
  //pinMode(Z_MS2, OUTPUT);
  //pinMode(Z_MS3, OUTPUT);
  
  
}

//////////////////////////////////////////////////////


void projectorRxChange() {cli(); projectorSerial.do_interrupt(); sei();}


///////////////////////////////////////////////////////
//
// setZero moves the build table to find the optical
// switch reference point.
//
void setZero (bool bFromAbove)
{
  setZSpeed(Z_RESETSPD);
  if (digitalRead(Z_HOME))
  {
    // Below zero
    do{
      if(digitalRead(Z_HOME) && iTargetPos != iCurPos + 1) iTargetPos = iCurPos + 1;
    }while (digitalRead(Z_HOME));
    do{delay(1);}while (iTargetPos != iCurPos);
    setZero(true); // Always find zero from above
    return;
  }
  else
  {
    // Above zero
    do{
      if(!digitalRead(Z_HOME) && iTargetPos != iCurPos - 1) iTargetPos = iCurPos - 1;
    }while (!digitalRead(Z_HOME));
    do{delay(1);}while (iTargetPos != iCurPos);
    if(!bFromAbove){
      setZero(false); 
      return;
    }
  }
  BC_C(F("Found Zero at:  "), String(iCurPos));
  BC_C(F("Reset Zero to:  "), String(iRefZeroOffset));
  BC_C(F("Difference of:  "), String(iCurPos - iRefZeroOffset));
  if(bReset) 
    BC_X(iCurPos - iRefZeroOffset);
   else
    BC_X(0);
    
  iCurPos = iRefZeroOffset;
  iTargetPos = iCurPos;
  setZSpeed(iZDriveSpeedCmd); 
  ZStepRelease();
}

///////////////////////////////////////////////////////


///////////////////////////////////////////////////////
//
// Drive iCurPos towards iTargetPos
// Drive iSlideCurPos towards iSlideTargetPos
// Called 800 times per second when z speed is 120 rpm
// Called 66.7 times per second when z speed is 10 rpm
// Called 933.3 times per second when z speed is 140 rpm
void updateMotors()
{
  if (iCurPos > iTargetPos) // We need to head down, -Z
    ZStep(Z_STEPDN);
  else if (iCurPos < iTargetPos) // We need to head up, +Z
    ZStep(Z_STEPUP);
  else if ((iCurPos == iTargetPos)&&(iZStatus==Z_MOVING))
  {
    ZStepRelease();
    iZStatus = Z_BROADCAST;
  }
  
  ///////////***************//////////////////////
  if(hangbing==1)
  {
      digitalWrite(SLIDE_MTR_DIR, 0);    
      analogWrite(SLIDE_MTR_PWM, X_ManualSPEED);
  }
    if(hangbing==2)
  {
      digitalWrite(SLIDE_MTR_DIR, 1);    
      analogWrite(SLIDE_MTR_PWM, X_ManualSPEED);
  }
    if(hangbing==0)
  {         
      analogWrite(SLIDE_MTR_PWM,0);
  }
  ///////////////******************//////////////////////
  
}

///////////////////////////////////////////////////////
//
// Reset the timer for stepper motor updates to
// control rpm
void setZSpeed(int iRPM){
  Timer1.setPeriod(150000/iRPM);
}

///////////////////////////////////////////////////////
//
// Release the Z Stepper Motor
//
void ZStepRelease(){
  digitalWrite(Z_ENABLE, LOW);//high
  digitalWrite(Z_DIR, LOW);
  digitalWrite(Z_STEP, LOW);
  lastStepPulse = LOW;
}

///////////////////////////////////////////////////////
//
// Step once, update position
//
void ZStep(int iDir){
  if(lastStepPulse == LOW){
    digitalWrite(Z_DIR, iDir);
    digitalWrite(Z_ENABLE, HIGH);//low
    digitalWrite(Z_STEP, HIGH);
    lastStepPulse = HIGH;
  }
  else {
    digitalWrite(Z_DIR, iDir);
    digitalWrite(Z_ENABLE, HIGH);//low
    digitalWrite(Z_STEP, LOW);
    lastStepPulse = LOW;
    if(iDir==Z_STEPDN) iCurPos--; else iCurPos++;
  }
}

///////////////////////////////////////////////////////
//
// Set the Z Stepper mode
//

void setZStepMode(int iMode)
{
  switch (iMode) 
  {
    default:
    case Z_STEP_FULL:
      digitalWrite(Z_MS1, LOW);
      digitalWrite(Z_MS2, LOW);
      digitalWrite(Z_MS3, LOW);
      break;
    case Z_STEP_HALF:
      digitalWrite(Z_MS1, HIGH);
      digitalWrite(Z_MS2, LOW);
      digitalWrite(Z_MS3, LOW);
      break;
    case Z_STEP_QTR:
      digitalWrite(Z_MS1, LOW);
      digitalWrite(Z_MS2, HIGH);
      digitalWrite(Z_MS3, LOW);
      break;
    case Z_STEP_8TH:
      digitalWrite(Z_MS1, HIGH);
      digitalWrite(Z_MS2, HIGH);
      digitalWrite(Z_MS3, LOW);
      break;
    case Z_STEP_16TH:
      digitalWrite(Z_MS1, HIGH);
      digitalWrite(Z_MS2, HIGH);
      digitalWrite(Z_MS3, HIGH);
      break;
  }
}


///////////////////////////////////////////////////////
//
// pull and return an int off the serial stream
//
int SerialReadInt() {
  // The string we read from Serial will be stored here:
  char str[32];
  str[0] = '\0';
  int i=0;
  while(true) {
    // See if we have serial data available:
    if (Serial.available() > 0) {
      // Yes, we have!
      // Store it at next position in string:
      str[i] = Serial.read();
      
      // If it is newline or we find a variable separator, then terminate the string
      // and leave the otherwise infinite loop:
      if (str[i] == '\n' || str[i] == '\0' || i==31) {
        str[i] = '\0';
        break;
      }
      // Ok, we were not at the end of the string, go on with next:
      else
        i++;
    }
  }
  // Convert the string to int and return:
  return(atoi(str));
} 


//////////////////////////////////////////////////////////////
//
// Handle the time based regular broadcasts
//  
void HandleTimedBroadcasts() {
  if(iBC_Interval > 0 && millis() - ulLastTimedBroadcastTime > iBC_Interval ) {
    ulLastTimedBroadcastTime = millis();

    BC_P();  // Broadcast Power Status
    BC_L();  // Broadcast Lamp Hours (if Power is on)
    
    // If we are in a print cycle and the exposure time exceeds the watchdog limit
    // Then we abort the print cycle
    if(iCycle != CYCLE_OFF && millis() - ulLastCmdReceivedTime > MAX_EXPOSURE_WATCHDOG)
    {
      BC_Q();  // Broadcast Print Cycle Abort Watchdog alert
      iCycle = CYCLE_OFF;  // abort the print cycle, Stop z axis motion, close the Shutter
      
      iCycle = CYCLE_OFF;
     // projectorSerial.write("V00S0002\r");  // Power down the projector
      //delay(1000);
      digitalWrite(PROJECTOR_ON,LOW);// TURN OFF PROJECTOR
      digitalWrite(LED_ON, LOW);
      delay(1000);
      while(lastStepPulse==HIGH){delay(1);} // wait until step LOW      
      iTargetPos = iCurPos; 
      bEnableManual = true;
    }
  }
}
  

//////////////////////////////////////////////////////////////
//
// Handle the automated projectors communications
//  Keep iProjectorPwr and iLampHours updated 
void HandleProjectorComm_Vivitek_D535() {
    
  if(millis() - ulLastProjectorMsgRcvdTime > ulLastProjectorMsgRcvdTime){
    //it's been 15 sec since we've heard from the projector!
    //set power unknown and reset connection
    bFirstProjectorQuery = true;
    iProjectorPwr = -1;
    iLampHours = -1;
   // projectorSerial.end();
   // projectorSerial.begin(9600);  
    // projectorSerial.write("V00G0007\r");  // Transmit Power status query
     //delay(1000);
     if(digitalRead(PROJECTOR_ON)==HIGH){
     iProjectorPwr = 1;
   }
     if(digitalRead(PROJECTOR_ON)==LOW){
     iProjectorPwr = 0;
     bProjectorNeedsReset = true;
   }
    ulLastProjectorMsgRcvdTime = millis();
    return;
  }
  
  if (!projectorSerial.available()){
    if(iProjectorPwr <0 || millis() - ulLastProjectorQueryTime > 5000  ) { // 5 second intervals
      // Time to ask if the projector is on...
      ulLastProjectorQueryTime = millis();
      if(bFirstProjectorQuery) {iProjectorPwr = 0; bFirstProjectorQuery=false;}
      // projectorSerial.write("V00G0007\r");  // Transmit Power status query
      // delay(1000);
       if(digitalRead(PROJECTOR_ON)==HIGH){
     iProjectorPwr = 1;
   }
 if(digitalRead(PROJECTOR_ON)==LOW){
     iProjectorPwr = 0;
     bProjectorNeedsReset = true;
   }
      
      
    }
    return;
  }
  ulLastProjectorMsgRcvdTime = millis();
  
  // Handle the projector's transmission
 /* String sData;
  char c;
  while(projectorSerial.available() > 0) {
    // Yes, we have data available
    c = projectorSerial.read();
    // If it is newline or we find a variable separator, then terminate
    // and leave the otherwise infinite loop, otherwise add it to sData
    if (c == '\r' || c == '\n' || c == '\0') break; else sData += String(c);
  }
  if(sData=="F") {
    iProjectorPwr = 0;
    bProjectorNeedsReset = true;
  }
  else if(sData=="P2") {
    iProjectorPwr = 1; 
    projectorSerial.write("V00G0004\r"); // Transmit Lamp hours query
  }  
  else if(sData=="hang" || sData=="bing") {
    sData=""; //ignore the "P" response
  }
  else {
    char str[65];
    sData.toCharArray(str,64);
    int iH = atoi(str);
    if(iH>=0 && iH<100000 && iH >= iLampHours) iLampHours = iH;
  }*/


   iLampHours =1000;//lamp hour 1000h
}

void projectorReset(){
  if(bProjectorNeedsReset){
    bProjectorNeedsReset = false;
  /*  projectorSerial.write("~rM\r");   //MENU
    delay(100);
    projectorSerial.write("~rL\r");   //LEFT
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN1
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN2
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN3
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN4
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN5
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN6
    delay(100);
    projectorSerial.write("~rD\r");   //DOWN7
    delay(100);
    projectorSerial.write("~rR\r");   //RIGHT 
    delay(1000);
    projectorSerial.write("~sB100\r");  // Vivitek D535 set brightness 100%
    projectorSerial.write("~sC100\r");  // Vivitek D535 set contrast 100%
    projectorSerial.write("~sR50\r");   // Vivitek D535 set Color 50%
    projectorSerial.write("~sN50\r");   // Vivitek D535 set Tint 50%
    projectorSerial.write("~sT1\r");    // Vivitek D535 set color Temperature Normal  ( 0:Cold 1:Normal 2:Warm )         
    projectorSerial.write("~sA4\r");    // Vivitek D535 set aspect (Scaling) to Native ( 0:Fill, 1:4to3, 2:16to9, 3:Letter Box, 4:Native )
    if(bIsMirrored)
      projectorSerial.write("~sJ1\r");  // Vivitek D535 set Projetion Mode to Back ( 0:Front 1:Rear 2:Rear+Ceiling 3:Ceiling )
      else
      projectorSerial.write("~sJ0\r");  // Vivitek D535 set Projetion Mode to Front ( 0:Front 1:Rear 2:Rear+Ceiling 3:Ceiling ) */           
  }
}  

//////////////////////////////////////////////////////////////
//
void loadEEPromSettings(){
  int schema;
  EEPROM_readAnything(EE_ADDRESS_SCHEMA, schema);

  // Set defaults
  iRefZeroOffset = ZHOMEOFFSETDEFAULT;
  iNativeX = XNATIVEDEFAULT;
  iNativeY = YNATIVEDEFAULT;
  iXYPixelSize = XYPSIZEDEFAULT;
  iHalfLife = HALFLIFEDEFAULT;
  
  if(schema >99 && schema < EE_SCHEMA) { // Load interesting old schema data first
    EEPROM_readAnything(EE_ADDRESS_Z_HOME, iRefZeroOffset);
    EEPROM_readAnything(EE_ADDRESS_NATIVEX, iNativeX);
    EEPROM_readAnything(EE_ADDRESS_NATIVEY, iNativeY);
    EEPROM_readAnything(EE_ADDRESS_XYPSIZE, iXYPixelSize);
    EEPROM_readAnything(EE_ADDRESS_HALFLIFE, iHalfLife);  
  }
  
  if(iNativeX <1024 || iNativeY <768) { //The interesting old data has been corrupted?  Restore defaults
    iRefZeroOffset = ZHOMEOFFSETDEFAULT;
    iNativeX = XNATIVEDEFAULT;
    iNativeY = YNATIVEDEFAULT;
    iXYPixelSize = XYPSIZEDEFAULT;
    iHalfLife = HALFLIFEDEFAULT;
    schema = -1; // force storeDefaults()
  }
  
  if(schema != EE_SCHEMA){
    storeDefaults();
  }
  else { // load current defaults
    EEPROM_readAnything(EE_ADDRESS_Z_HOME, iRefZeroOffset);
    EEPROM_readAnything(EE_ADDRESS_NATIVEX, iNativeX);
    EEPROM_readAnything(EE_ADDRESS_NATIVEY, iNativeY);
    EEPROM_readAnything(EE_ADDRESS_XYPSIZE, iXYPixelSize);
    EEPROM_readAnything(EE_ADDRESS_HALFLIFE, iHalfLife);
    // Add more default loads here...
  }
}

void storeDefaults(){
  // Default never burned.  Store the defaults
  EEPROM_writeAnything(EE_ADDRESS_SCHEMA, EE_SCHEMA);
  EEPROM_writeAnything(EE_ADDRESS_Z_HOME, iRefZeroOffset);
  EEPROM_writeAnything(EE_ADDRESS_NATIVEX, iNativeX);
  EEPROM_writeAnything(EE_ADDRESS_NATIVEY, iNativeY);
  EEPROM_writeAnything(EE_ADDRESS_XYPSIZE, iXYPixelSize);
  EEPROM_writeAnything(EE_ADDRESS_HALFLIFE, iHalfLife);
  // Add more defaults here... Make sure address accounts for size of data stored
}

void resetFactoryDefaults(){
  iRefZeroOffset = ZHOMEOFFSETDEFAULT;
  iNativeX = XNATIVEDEFAULT;
  iNativeY = YNATIVEDEFAULT;
  iXYPixelSize = XYPSIZEDEFAULT;
  iHalfLife = HALFLIFEDEFAULT;
  storeDefaults();
}

void storeRefZOffset(){
    EEPROM_writeAnything(EE_ADDRESS_Z_HOME, iRefZeroOffset);
}
void storeRefNativeX(){
    EEPROM_writeAnything(EE_ADDRESS_NATIVEX, iNativeX);
}
void storeRefNativeY(){
    EEPROM_writeAnything(EE_ADDRESS_NATIVEY, iNativeY);
}
void storeRefXYPixelSize(){
    EEPROM_writeAnything(EE_ADDRESS_XYPSIZE, iXYPixelSize);
}
void storeHalfLife(){
    EEPROM_writeAnything(EE_ADDRESS_HALFLIFE, iHalfLife);
}

//////////////////////////////////////////////////////////////
//
// Broadcast Commands
//

//////////////////////////////////////////////////////////////
void BC_String(String sString) {
  Serial.println(sString);
}

//////////////////////////////////////////////////////////////
void BC_String(const __FlashStringHelper* s, String sVariable) {
    Serial.print(s);
    Serial.println(sVariable);
}

//////////////////////////////////////////////////////////////
void BC_A() {
  BC_String(F("A1"));
  BC_C(F("Projector Control:  Available"));
}

//////////////////////////////////////////////////////////////
void BC_C(String sComment) {
  if(bVerbose){
    BC_String("C" + sComment);
  }
}
void BC_C(const __FlashStringHelper* s, String sVariable) {
  if(bVerbose){
    Serial.print("C");
    Serial.print(s);
    Serial.println(sVariable);
  }
}

//////////////////////////////////////////////////////////////
void BC_D() {
  BC_String(F("D"), String(iNativeX));
  BC_C(F("Projector's Native X Resolution: "), String(iNativeX));
}

//////////////////////////////////////////////////////////////
void BC_E() {
  BC_String(F("E"), String(iNativeY));
  BC_C(F("Projector's Native Y Resolution: "), String(iNativeY));
}

//////////////////////////////////////////////////////////////
void BC_F() {
  BC_String(F("F"));
  BC_C(F("Cycle Finished."));
}

//////////////////////////////////////////////////////////////
void BC_H() {
  BC_String(F("H"), String(iXYPixelSize));
  BC_C(F("Calibrated XY Pixel Size in Microns: "), String(iXYPixelSize));
}

//////////////////////////////////////////////////////////////
void BC_I() {
  BC_String(F("I500"));
  BC_C(F("Printer Unit:500"));
}

//////////////////////////////////////////////////////////////
void BC_J() {
  BC_String(F("J1"));
  BC_C(F("Shutter Control: Available"));
}

//////////////////////////////////////////////////////////////
void BC_K() {
  BC_String(F("K"), String(iHalfLife));
  BC_C(F("Estimated hours before lamp output is 1/2 of original value: "), String(iHalfLife));
}

//////////////////////////////////////////////////////////////
void BC_L() {
  if(iLampHours>=0 && iProjectorPwr>0) {
    BC_String(F("L"), String(iLampHours));
    BC_C(F("Lamp Hours: "), String(iLampHours));
  } 
}

//////////////////////////////////////////////////////////////
void BC_M() {
  BC_String(F("M"), String(iUpperZLimit));
  BC_C(F("Maximum Z in PU: "),String(iUpperZLimit));
}

//////////////////////////////////////////////////////////////
void BC_P() {
  if(iProjectorPwr<1) {
    BC_String(F("P0"));
    BC_C(F("Projector is Off"));
  } 
  else {
    BC_String(F("P1"));
    BC_C(F("Projector is On"));
  }
 
}

//////////////////////////////////////////////////////////////
void BC_Q() {
  BC_String(F("Q"));
  BC_C(F("Print Cycle ABORT.  Host COMM Lost"));
}

//////////////////////////////////////////////////////////////
void BC_R() {
  if(bReset)
  {
    BC_String(F("R0"));
    BC_C(F("Needs Reset: NO"));
  }
  else {
    BC_String(F("R1"));
    BC_C(F("Needs Reset: YES"));
  }
}

//////////////////////////////////////////////////////////////
void BC_S(int ipercentopen) {
BC_String(F("S"),String(ipercentopen));
BC_C(F("Current Slide Percent Open is:"),String(ipercentopen));
}

//////////////////////////////////////////////////////////////
void BC_U(int iFault) {
  BC_String(F("U"), String(iFault));
  if(iFault == 1) BC_C(F("ERROR:  Runaway X Mtr, Counter Clockwise"));
  if(iFault == 2) BC_C(F("ERROR:  Runaway X Mtr, Clockwise"));
  if(iFault == 3) BC_C(F("ERROR:  Runaway X Mtr, Bad Sensor"));
}

//////////////////////////////////////////////////////////////
void BC_V() {
    BC_String(F("V9 9 9"));
    BC_String(F("WGENIUSCREATION"));
    BC_C(F("B9Creator(tm) Firmware version 9.9.9 running on a B9Creator Model 1"));
    BC_C(F("Copyright 2012, 2013 B9Creations, LLC"));
    BC_C(F("B9Creations(tm) and B9Creator(tm) are trademarks of B9Creations, LLC"));
    BC_C(F("This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License."));
    BC_C(F("To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/"));
    BC_C(F("For updates and to download the lastest version, visit http://b9creator.com"));
}

//////////////////////////////////////////////////////////////
void BC_X(int iDiff) {
  BC_String(F("X"), String(iDiff));
  BC_C(F("Reset Compelete with Z Diff of:"),String(iDiff));
}

//////////////////////////////////////////////////////////////
void BC_Y() {
  BC_String(F("Y"), String(iRefZeroOffset));
  BC_C(F("Current Z Home Value:"),String(iRefZeroOffset));
}

//////////////////////////////////////////////////////////////
void BC_Z() {
  BC_String(F("Z"), String(iCurPos));
  BC_C(F("Current Z Position is: "), String(iCurPos));
}


