//====================================================================
//Project Lynxmotion Phoenix
//
// Servo Driver - This version is setup to use AX-12 type servos using the
// Arbotix AX12 and bioloid libraries (which may have been updated)
//====================================================================
#include <Arduino.h>
#include "Hex_Cfg.h"
#include <LSS.h>
#include "phoenix_float.h"
#include "phoenix_driver_bioloid.h"

#ifdef c4DOF
#define NUMSERVOSPERLEG 4
#else
#define NUMSERVOSPERLEG 3
#endif

#ifdef cTurretRotPin
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS +2)
#else
#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)
#endif

#define cPwmMult      128
#define cPwmDiv       375
#define cPFConst      512    // half of our 1024 range

// Some defines for Voltage processing
#define VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE 4000  // Min time in us Until we should do next interpolation, as to not interfer.
#define VOLTAGE_MIN_TIME_BETWEEN_CALLS 150      // Max 6+ times per second
#define VOLTAGE_MAX_TIME_BETWEEN_CALLS 1000    // call at least once per second...
#define VOLTAGE_TIME_TO_ERROR          3000    // Error out if no valid item is returned in 3 seconds...


// Current positions in AX coordinates
int16_t      g_cur_servo_pos[NUMSERVOS];
int16_t      g_goal_servo_pos[NUMSERVOS];

#ifdef DBGSerial
//#define DEBUG
// Only allow debug stuff to be turned on if we have a debug serial port to output to...
#define DEBUG_SERVOS
#endif

#ifdef DEBUG_SERVOS
#define ServosEnabled   (g_fEnableServos)
#else
#define ServosEnabled  (true)      // always true compiler should remove if...
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
static const char *lss_status_text[] PROGMEM  = {
	"Unknown",	"Limp",	"FreeMoving",	"Accelerating",	"Travelling",	"Decelerating",
	"Holding",	"OutsideLimits",	"Stuck",  "Blocked",
	"SafeMode", "Last"
};

static const byte cPinTable[] = {
	cRRCoxaPin,  cRMCoxaPin,  cRFCoxaPin,  cLRCoxaPin,  cLMCoxaPin,  cLFCoxaPin,
	cRRFemurPin, cRMFemurPin, cRFFemurPin, cLRFemurPin, cLMFemurPin, cLFFemurPin,
	cRRTibiaPin, cRMTibiaPin, cRFTibiaPin, cLRTibiaPin, cLMTibiaPin, cLFTibiaPin
#ifdef c4DOF
	, cRRTarsPin, cRMTarsPin, cRFTarsPin, cLRTarsPin, cLMTarsPin, cLFTarsPin
#endif
#ifdef cTurretRotPin
	, cTurretRotPin, cTurretTiltPin
#endif
};

#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif
// Not sure yet if I will use the controller class or not, but...
LSS myLSS = LSS(0);
boolean g_fServosFree;    // Are the servos in a free state?

// BUGBUG: // tired of some of the wrappers... create quick and dirty
inline int32_t GetServoPosition(uint8_t servo) {
	myLSS.setServoID(servo);
	return myLSS.getPosition();
}



//=============================================================================
// Global - Local to this file only...
//=============================================================================

#define FIRSTCOXAPIN     0
#define FIRSTFEMURPIN    (CNT_LEGS)
#define FIRSTTIBIAPIN    (CNT_LEGS*2)
#ifdef c4DOF
#define FIRSTTARSPIN     (CNT_LEGS*3)
#define FIRSTTURRETPIN   (CNT_LEGS*4)
#else
#define FIRSTTURRETPIN   (CNT_LEGS*3)
#endif

//====================================
//set MJS RF config Gait Test Values
// and Mucked up by KJE ;)
//====================================
typedef struct {
	uint8_t         id;
	LSS_ConfigGyre  gyre;
	int16_t         offset;
	int16_t         max_speed;
	LSS_Status      move_status;
	int32_t         time_position;
} servo_info_t;
typedef struct {
	const char    *leg_name;
	servo_info_t  coxa;
	servo_info_t  femur;
	servo_info_t  tibia;
	bool          leg_found;
} leg_info_t;

leg_info_t legs[] = {
	{"Left Front",  {cLFCoxaPin, LSS_GyreClockwise, 0, 600}, {cLFFemurPin, LSS_GyreClockwise, -104, 600}, {cLFTibiaPin, LSS_GyreClockwise, -137, 600}},
	{"Left Middle", {cLMCoxaPin, LSS_GyreClockwise, 0, 600}, {cLMFemurPin, LSS_GyreClockwise, -104, 600}, {cLMTibiaPin, LSS_GyreClockwise, -137, 600}},
	{"Left Rear",   {cLRCoxaPin, LSS_GyreClockwise, 0, 600}, {cLRFemurPin, LSS_GyreClockwise, -104, 600}, {cLRTibiaPin, LSS_GyreClockwise, -137, 600}},

	{"Right Front",  {cRFCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRFFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRFTibiaPin, LSS_GyreCounterClockwise, -137, 600}},
	{"Right Middle", {cRMCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRMFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRMTibiaPin, LSS_GyreCounterClockwise, -137, 600}},
	{"Right Rear",   {cRRCoxaPin, LSS_GyreCounterClockwise, 0, 600}, {cRRFemurPin, LSS_GyreCounterClockwise, -104, 600}, {cRRTibiaPin, LSS_GyreCounterClockwise, -137, 600}}
};
#define COUNT_LEGS (sizeof(legs)/sizeof(legs[0]))

void LSSServoDriver::setGaitConfig()
{
	for (uint8_t leg = 0; leg < COUNT_LEGS; leg++) {
		legs[leg].leg_found = true;
		myLSS.setServoID(legs[leg].coxa.id);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		myLSS.setMaxSpeed(legs[leg].coxa.max_speed, LSS_SetSession);
		myLSS.setGyre(legs[leg].coxa.gyre, LSS_SetSession);
		myLSS.setOriginOffset(legs[leg].coxa.offset, LSS_SetSession);
		myLSS.setMotionControlEnabled(0);
		myLSS.setAngularHoldingStiffness(-4);

		myLSS.setServoID(legs[leg].femur.id);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		myLSS.setMaxSpeed(legs[leg].femur.max_speed, LSS_SetSession);
		myLSS.setGyre(legs[leg].femur.gyre, LSS_SetSession);
		myLSS.setOriginOffset(legs[leg].femur.offset, LSS_SetSession);
		myLSS.setMotionControlEnabled(0);
		myLSS.setAngularHoldingStiffness(-4);

		myLSS.setServoID(legs[leg].tibia.id);
		if (myLSS.getStatus() == LSS_StatusUnknown) legs[leg].leg_found = false;
		myLSS.setMaxSpeed(legs[leg].tibia.max_speed, LSS_SetSession);
		myLSS.setGyre(legs[leg].tibia.gyre, LSS_SetSession);
		myLSS.setOriginOffset(legs[leg].tibia.offset, LSS_SetSession);
		myLSS.setMotionControlEnabled(0);
		myLSS.setAngularHoldingStiffness(-4);

		if (legs[leg].leg_found) Serial.printf("Servos for Leg %s **found**\n", legs[leg].leg_name);
		else Serial.printf("Servos for Leg %s **NOT found**\n", legs[leg].leg_name);
	}
}

//--------------------------------------------------------------------
//Init
//--------------------------------------------------------------------
void LSSServoDriver::Init(void) {
	// First lets get the actual servo positions for all of our servos...
	//  pinMode(0, OUTPUT);
	LSS::initBus(LSS_SERIAL_PORT, LSS_BAUD);
#ifdef LSS_SupportsSettingTimeouts
	LSS::setReadTimeouts(20, 5); // define we will wait for 20ms for response to start and 5ms for in message characters
#endif

	g_fServosFree = true;
#ifdef DBGSerial
	int32_t pos;
	int     count_missing = 0;

	for (int i = 0; i < NUMSERVOS; i++) {
		// Set the id
		int servo_id = pgm_read_byte(&cPinTable[i]);
		uint8_t index = TMAddID(servo_id); // bugbug assuming some position stuff...
		myLSS.setServoID(servo_id);

		// Now try to get it's position
		DBGSerial.print("Servo(");
		DBGSerial.print(i, DEC);
		DBGSerial.print("): ");
		DBGSerial.print(servo_id, DEC);

		pos = myLSS.getPosition();
		if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
			DBGSerial.printf(" %d\n", pos);
			TMSetTargetByIndex(index, pos);
		}
		else {
			DBGSerial.println(" not found");
			++count_missing;
		}
		delay(25);
	}

	// Now see if we should try to recover from a potential servo that renumbered itself back to 1.
	if (count_missing) {
		DBGSerial.print("ERROR: Servo driver init: ");
		DBGSerial.print(count_missing, DEC);
		DBGSerial.println(" servos missing");
	}
#endif


#ifdef cVoltagePin
	for (byte i = 0; i < 8; i++)
		GetBatteryVoltage();  // init the voltage pin
#endif

	// Hack to setup servos... 
  setGaitConfig(); // should phase out

}


//--------------------------------------------------------------------
//GetBatteryVoltage - Maybe should try to minimize when this is called
// as it uses the serial port... Maybe only when we are not interpolating
// or if maybe some minimum time has elapsed...
//--------------------------------------------------------------------

#ifdef cVoltagePin
word  g_awVoltages[8] = {
	0, 0, 0, 0, 0, 0, 0, 0
};
word  g_wVoltageSum = 0;
byte  g_iVoltages = 0;

word LSSServoDriver::GetBatteryVoltage(void) {
	g_iVoltages = (g_iVoltages + 1) & 0x7; // setup index to our array...
	g_wVoltageSum -= g_awVoltages[g_iVoltages];
	g_awVoltages[g_iVoltages] = analogRead(cVoltagePin);
	g_wVoltageSum += g_awVoltages[g_iVoltages];

#ifdef CVREF
	return ((long)((long)g_wVoltageSum * CVREF * (CVADR1 + CVADR2)) / (long)(8192 * (long)CVADR2));
#else
	return ((long)((long)g_wVoltageSum * 125 * (CVADR1 + CVADR2)) / (long)(2048 * (long)CVADR2));
#endif
}

#else
word g_wLastVoltage = 0xffff;    // save the last voltage we retrieved...
byte g_bLegVoltage = 0;   // what leg did we last check?
unsigned long g_ulTimeLastBatteryVoltage;

word LSSServoDriver::GetBatteryVoltage(void) {
	// In this case, we have to ask a servo for it's current voltage level, which is a lot more overhead than simply doing
	// one AtoD operation.  So we will limit when we actually do this to maybe a few times per second.
	// Also if interpolating, the code will try to only call us when it thinks it won't interfer with timing of interpolation.
#ifdef LATER
	unsigned long ulDeltaTime = millis() - g_ulTimeLastBatteryVoltage;
	if (g_wLastVoltage != 0xffff) {
		if ((ulDeltaTime < VOLTAGE_MIN_TIME_BETWEEN_CALLS)
		    || (bioloid.interpolating && (ulDeltaTime < VOLTAGE_MAX_TIME_BETWEEN_CALLS)))
			return g_wLastVoltage;
	}

	// Lets cycle through the Tibia servos asking for voltages as they may be the ones doing the most work...
	register word wVoltage = ax12GetRegister(pgm_read_byte(&cPinTable[FIRSTTIBIAPIN + g_bLegVoltage]), AX_PRESENT_VOLTAGE, 1);
	if (++g_bLegVoltage >= CNT_LEGS)
		g_bLegVoltage = 0;
	if (wVoltage != 0xffff) {
		g_ulTimeLastBatteryVoltage = millis();
		g_wLastVoltage = wVoltage * 10;
		return g_wLastVoltage;
	}

	// Allow it to error our a few times, but if the time until we get a valid response exceeds some time limit then error out.
	if (ulDeltaTime < VOLTAGE_TIME_TO_ERROR)
		return g_wLastVoltage;
#endif
	return 0;

}
#endif


//------------------------------------------------------------------------------------------
//[BeginServoUpdate] Does whatever preperation that is needed to starrt a move of our servos
//------------------------------------------------------------------------------------------
void LSSServoDriver::BeginServoUpdate(void)    // Start the update
{
	MakeSureServosAreOn();

}

//------------------------------------------------------------------------------------------
//[OutputServoInfoForLeg] Do the output to the SSC-32 for the servos associated with
//         the Leg number passed in.
//------------------------------------------------------------------------------------------
#ifdef c4DOF
void LSSServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1)
#else
void LSSServoDriver::OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1)
#endif
{
	// Save away the new positions...
	// Sort of duplicate stuff now... May clean up.
	TMSetTargetByIndex(FIRSTCOXAPIN + LegIndex, sCoxaAngle1);  // What order should we store these values?
	TMSetTargetByIndex(FIRSTFEMURPIN + LegIndex, sFemurAngle1);
	TMSetTargetByIndex(FIRSTTIBIAPIN + LegIndex, sTibiaAngle1);
#ifdef c4DOF
	TMSetTargetByIndex(FIRSTTARSPIN + LegIndex, sTarsAngle);
#endif
	g_goal_servo_pos[FIRSTCOXAPIN + LegIndex] = sCoxaAngle1;  // What order should we store these values?
	g_goal_servo_pos[FIRSTFEMURPIN + LegIndex] = sFemurAngle1;
	g_goal_servo_pos[FIRSTTIBIAPIN + LegIndex] = sTibiaAngle1;
#ifdef c4DOF
	g_awGoalAXTarsPos[FIRSTTARSPIN + LegIndex] = sTarsAngle1;
#endif
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput) {
		DBGSerial.print(LegIndex, DEC);
		DBGSerial.print("(C");
		DBGSerial.print(sCoxaAngle1, DEC);
		DBGSerial.print("),(F");
		DBGSerial.print(sFemurAngle1, DEC);
		DBGSerial.print("),(");
		DBGSerial.print("T(");
		DBGSerial.print(sTibiaAngle1, DEC);
		DBGSerial.print(") :");
	}
#endif
	InputController::controller()->AllowControllerInterrupts(true);    // Ok for hserial again...
}


//------------------------------------------------------------------------------------------
//[OutputServoInfoForTurret] Set up the outputse servos associated with an optional turret
//         the Leg number passed in.  FIRSTTURRETPIN
//------------------------------------------------------------------------------------------
#ifdef cTurretRotPin
void LSSServoDriver::OutputServoInfoForTurret(short sRotateAngle1, short sTiltAngle1)
{
#ifdef LATER
	word    wRotateSDV;
	word    wTiltSDV;        //

	// The Main code now takes care of the inversion before calling.
	wRotateSDV = (((long)(sRotateAngle1)) * cPwmMult) / cPwmDiv + cPFConst;
	wTiltSDV = (((long)((long)(sTiltAngle1)) * cPwmMult) / cPwmDiv + cPFConst);

	if (ServosEnabled) {
		if (g_fAXSpeedControl) {
#ifdef USE_AX12_SPEED_CONTROL
			// Save away the new positions...
			g_goal_servo_pos[FIRSTTURRETPIN] = wRotateSDV;    // What order should we store these values?
			g_goal_servo_pos[FIRSTTURRETPIN + 1] = wTiltSDV;
#endif
		}
		else {
			bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN]), wRotateSDV);
			bioloid.setNextPose(pgm_read_byte(&cPinTable[FIRSTTURRETPIN + 1]), wTiltSDV);
		}
	}
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput) {
		DBGSerial.print("(");
		DBGSerial.print(sRotateAngle1, DEC);
		DBGSerial.print("=");
		DBGSerial.print(wRotateSDV, DEC);
		DBGSerial.print("),(");
		DBGSerial.print(sTiltAngle1, DEC);
		DBGSerial.print("=");
		DBGSerial.print(wTiltSDV, DEC);
		DBGSerial.print(") :");
	}
#endif
#endif
}
#endif
//--------------------------------------------------------------------
//[CommitServoDriver Updates the positions of the servos - This outputs
//         as much of the command as we can without committing it.  This
//         allows us to once the previous update was completed to quickly
//        get the next command to start
//--------------------------------------------------------------------
void LSSServoDriver::CommitServoDriver(word wMoveTime)
{
	InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...
	if (ServosEnabled) {
		if (use_servos_timed_moves) {
			for (int i = 0; i < NUMSERVOS; i++) {
				if (g_cur_servo_pos[i] != g_goal_servo_pos[i]) {
					g_cur_servo_pos[i] = g_goal_servo_pos[i];
					// Set the id
					int servo_id = pgm_read_byte(&cPinTable[i]);
					myLSS.setServoID(servo_id);
					myLSS.moveT(g_goal_servo_pos[i] * 10, wMoveTime);
				}
			}
		} else {
			TMSetupMove(wMoveTime);
			if (g_fDebugOutput || servo_debug) TMPrintDebugInfo();
			TMStep(true); // force the first step..
		}
	} else {
		// Rear middle front
		//DBGSerial.println("Servo positions shown by leg joints\n(Rear)");
		//DBGSerial.println("    T     F     C |     C     F     T");
		for (int legs = 0; legs < 3; legs++) {
			DBGSerial.printf("%5d %5d %5d | %5d %5d %5d || ",
			                 g_goal_servo_pos[FIRSTTIBIAPIN + legs], g_goal_servo_pos[FIRSTFEMURPIN + legs], g_goal_servo_pos[FIRSTCOXAPIN + legs],
			                 g_goal_servo_pos[FIRSTCOXAPIN + legs + 3], g_goal_servo_pos[FIRSTFEMURPIN + legs + 3], g_goal_servo_pos[FIRSTTIBIAPIN + legs + 3]);
		}
		Serial.printf("%u\n", wMoveTime);
	}
#ifdef DEBUG_SERVOS
	if (g_fDebugOutput)
		DBGSerial.println(wMoveTime, DEC);
#endif
	InputController::controller()->AllowControllerInterrupts(true);
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
//--------------------------------------------------------------------
void LSSServoDriver::FreeServos(void)
{
	if (!g_fServosFree) {
		InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

		// See if we can do by broadcast
		LSS::genericWrite(LSS_BroadcastID, LSS_ActionLimp); // Tell all of the servos to go limp
		InputController::controller()->AllowControllerInterrupts(true);
		g_fServosFree = true;
	}
}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
static uint8_t g_iIdleServoNum = (uint8_t) - 1;
static uint8_t g_iIdleLedState = 1;  // what state to we wish to set...
void LSSServoDriver::IdleTime(void)
{
	// Each time we call this set servos LED on or off...
	// Lets just have one on at a time.
	if (g_iIdleServoNum < NUMSERVOS) {
		myLSS.setServoID(cPinTable[g_iIdleServoNum]);
		myLSS.setColorLED(LSS_LED_Black);
	}

	g_iIdleServoNum++;
	if (g_iIdleServoNum >= NUMSERVOS) {
		g_iIdleServoNum = 0;
		g_iIdleLedState++;
		if (g_iIdleLedState > 7) g_iIdleLedState = 0;
	}
	myLSS.setServoID(cPinTable[g_iIdleServoNum]);
	myLSS.setColorLED((LSS_LED_Color)g_iIdleLedState);

}

//--------------------------------------------------------------------
//Function that gets called from the main loop if the robot is not logically
//     on.  Gives us a chance to play some...
//--------------------------------------------------------------------
void LSSServoDriver::showUserFeedback(int feedback_state) {
	switch (feedback_state) {
	case 0:
		// turn off all leds...
		LSS::genericWrite(LSS_BroadcastID, LSS_ActionColorLED, LSS_LED_Black);
		break;
	}
}

//--------------------------------------------------------------------
//[MakeSureServosAreOn] Function that is called to handle when you are
//  transistioning from servos all off to being on.  May need to read
//  in the current pose...
//--------------------------------------------------------------------
void LSSServoDriver::MakeSureServosAreOn(void)
{
	if (ServosEnabled) {
		if (!g_fServosFree)
			return;    // we are not free

		InputController::controller()->AllowControllerInterrupts(false);    // If on xbee on hserial tell hserial to not processess...

		LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position.
		boolean servos_reset = false;
		for (int i = 0; i < NUMSERVOS; i++) {
			g_cur_servo_pos[i] = 32768; // set to a value that is not valid to force next output
			// lets make sure that servos are not in an error state.
			myLSS.setServoID(cPinTable[i]);
			LSS_Status servo_status = myLSS.getStatus();
			switch (servo_status) {
			case LSS_StatusUnknown:
			case LSS_StatusHolding:
				break;	// don't need to do anything
			case LSS_StatusLimp:
			case LSS_StatusFreeMoving:
			case LSS_StatusAccelerating:
			case LSS_StatusTravelling:
			case LSS_StatusDecelerating:
			case LSS_StatusOutsideLimits:
			case LSS_StatusStuck:
			case LSS_StatusBlocked:
			case LSS_StatusSafeMode:
			default:
				DBGSerial.printf("EnableServos: Servo %d reset due to status: %s(%d)\n", cPinTable[i],
				                 (servo_status < (sizeof(lss_status_text) / sizeof(lss_status_text[0]))) ? lss_status_text[servo_status] : "?",
				                 servo_status);
				myLSS.reset();
				servos_reset = true;
				break;
			}
		}

		if (servos_reset) {
			delay(3000);  // give servos some time to reset.
			// try again to hold servos.
			LSS::genericWrite(LSS_BroadcastID, LSS_ActionHold); // Tell all of the servos to hold a position
		}

		InputController::controller()->AllowControllerInterrupts(true);
		g_fServosFree = false;
	}
}

//==============================================================================
// BackgroundProcess - Allows us to have some background processing for those
//    servo drivers that need us to do things like polling...
//==============================================================================
void  LSSServoDriver::BackgroundProcess(void)
{
#ifdef cTurnOffVol          // only do if we a turn off voltage is defined
#ifndef cVoltagePin         // and we are not doing AtoD type of conversion...
	if (iTimeToNextInterpolate > VOLTAGE_MIN_TIME_UNTIL_NEXT_INTERPOLATE)      // At least 4ms until next interpolation.  See how this works...
		GetBatteryVoltage();
#endif
#endif
}


#ifdef OPT_TERMINAL_MONITOR
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void LSSServoDriver::ShowTerminalCommandList(void)
{
	DBGSerial.println(F("V - Voltage"));
	DBGSerial.println(F("M - Toggle Motors on or off"));
	DBGSerial.println(F("T - Test Servos"));
	DBGSerial.println(F("P - Servo Positions"));
	DBGSerial.println(F("S - Track Servos"));
	DBGSerial.println(F("A - Toggle LSS speed control"));
	DBGSerial.println(F("L - Toggle LSS Servo Debug output"));
	DBGSerial.println(F("F <FPS> - Set FPS for Interpolation mode"));
#ifdef OPT_PYPOSE
	DBGSerial.println(F("P<DL PC> - Pypose"));
#endif
#ifdef OPT_FIND_SERVO_OFFSETS
	DBGSerial.println(F("O - Enter Servo offset mode"));
#endif
}

//==============================================================================
// ProcessTerminalCommand: The terminal monitor will call this to see if the
//     command the user entered was one added by the servo driver.
//==============================================================================
boolean LSSServoDriver::ProcessTerminalCommand(byte* psz, byte bLen)
{
	if ((bLen == 1) && ((*psz == 'm') || (*psz == 'M'))) {
		g_fEnableServos = !g_fEnableServos;
		if (g_fEnableServos) {

			DBGSerial.println(F("Motors are on"));
		}
		else {
			DBGSerial.println(F("Motors are off"));
			FreeServos();	// make sure we turn off servos.
		}

		return true;
	}
	if ((bLen == 1) && ((*psz == 'v') || (*psz == 'V'))) {
		DBGSerial.print(F("Voltage: "));
		DBGSerial.println(GetBatteryVoltage(), DEC);
#ifdef cVoltagePin
		DBGSerial.print("Raw Analog: ");
		DBGSerial.println(analogRead(cVoltagePin));
#endif

		DBGSerial.print(F("From Servo 2: "));
		myLSS.setServoID(2);
		DBGSerial.println(myLSS.getVoltage(), DEC);
	}

	if ((bLen == 1) && ((*psz == 't') || (*psz == 'T'))) {
		// Test to see if any servos are responding
		for (int i = 1; i <= 32; i++) {
			int iPos;
			myLSS.setServoID(i);
			iPos = myLSS.getPosition();
			DBGSerial.print(i, DEC);
			DBGSerial.print(F("="));
			if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
				DBGSerial.println(iPos, DEC);
			}
			else {
				DBGSerial.println(F("** failed **"));
			}
			delay(25);
		}
	}
	if (((*psz == 'p') || (*psz == 'P'))) {
		TCServoPositions();
	}
	if ((*psz == 's') || (*psz == 'S')) {
		TCTrackServos();
	}
	else if ((bLen == 1) && ((*psz == 'a') || (*psz == 'A'))) {
		use_servos_timed_moves = !use_servos_timed_moves;
		if (use_servos_timed_moves) {
			DBGSerial.println(F("Use Servo moveT"));
		}
		else {
			DBGSerial.println(F("Use Software Interplation"));
		}
		TMConfigureServos();
		return true;
	}

	else if ((bLen == 1) && ((*psz == 'l') || (*psz == 'L'))) {
		servo_debug = !servo_debug;
		if (servo_debug) {
			DBGSerial.println(F("LSS Debug output enabled"));
		}
		else {
			DBGSerial.println(F("LSS Debug output disabled"));
		}
		return true;
	}
	else if ((*psz == 'f') || (*psz == 'F')) {
		uint16_t fps = 0;
		psz++;
		while (*psz == ' ') psz++; // ignore any blanks.
		while ((*psz >= '0') && (*psz <= '9')) {
			fps = fps * 10 + *psz++ - '0';
		}
		if (fps == 0) fps = DEFAULT_FRAMES_PER_SECOND;
		tmCycleTime = 1000000 / fps;
		DBGSerial.printf("Set FPS to: %u Cycle time\n", fps, tmCycleTime);
	}



#ifdef OPT_FIND_SERVO_OFFSETS
	else if ((bLen == 1) && ((*psz == 'o') || (*psz == 'O'))) {
		FindServoOffsets();
	}
#endif
	return false;

}
//==============================================================================
// TCServoPositions -
//==============================================================================
void LSSServoDriver::TCServoPositions() {
	int16_t servo_pos[NUMSERVOS];
	LSS_Status servo_status[NUMSERVOS];
	int i;

	for (i = 0; i < NUMSERVOS; i++) {
		myLSS.setServoID(cPinTable[i]);
		servo_pos[i] = myLSS.getPosition();
		LSS_LastCommStatus lss_status = myLSS.getLastCommStatus();
		if (lss_status != LSS_CommStatus_ReadSuccess) {
			servo_pos[i] = 0x7fff; // out of valid range
			DBGSerial.printf("%u fail: %x\n", cPinTable[i], (uint32_t)lss_status);
		}
		servo_status[i] = myLSS.getStatus();
	}

	// Not very clean
	// Rear middle front
	DBGSerial.println("Servo positions shown by leg joints\n(Rear)");
	DBGSerial.println("    T     F     C |     C     F     T");
	for (int legs = 0; legs < 3; legs++) {
		DBGSerial.printf("%5d(%u) %5d(%u) %5d(%u) | %5d(%u) %5d(%u) %5d(%u)\n",
		                 servo_pos[FIRSTTIBIAPIN + legs], 	 servo_status[FIRSTTIBIAPIN + legs],
		                 servo_pos[FIRSTFEMURPIN + legs], 	 servo_status[FIRSTFEMURPIN + legs],
		                 servo_pos[FIRSTCOXAPIN + legs], 	 servo_status[FIRSTCOXAPIN + legs],
		                 servo_pos[FIRSTCOXAPIN + legs + 3],  servo_status[FIRSTCOXAPIN + legs + 3],
		                 servo_pos[FIRSTFEMURPIN + legs + 3], servo_status[FIRSTFEMURPIN + legs + 3],
		                 servo_pos[FIRSTTIBIAPIN + legs + 3], servo_status[FIRSTTIBIAPIN + legs + 3]);
	}
}

//==============================================================================
// TCTrackServos - Lets set a mode to track servos.  Can use to help figure out
// proper initial positions and min/max values...
//==============================================================================
void LSSServoDriver::TCTrackServos()
{
	// First read through all of the servos to get their position.
	int16_t auPos[NUMSERVOS];
	int16_t  uPos;
	int16_t servo_mins[NUMSERVOS];
	int16_t servo_maxs[NUMSERVOS];
	int i;
	boolean fChange;

	// Clear out any pending input characters
	while (DBGSerial.read() != -1)
		;
	DBGSerial.println("\nTrack servos - enter any key to exit");
	for (i = 0; i < NUMSERVOS; i++) {
		myLSS.setServoID(cPinTable[i]);
		servo_mins[i] = auPos[i] = servo_maxs[i] = myLSS.getPosition();
	}

	// Now loop until we get some input on the serial
	while (!DBGSerial.available()) {
		fChange = false;
		for (int i = 0; i < NUMSERVOS; i++) {
			myLSS.setServoID(cPinTable[i]);
			uPos = myLSS.getPosition();
			if (myLSS.getLastCommStatus() == LSS_CommStatus_ReadSuccess) {
				if (uPos > servo_maxs[i]) servo_maxs[i] = uPos;
				if (uPos < servo_mins[i]) servo_mins[i] = uPos;
				// Lets put in a littl delta or shows lots
				if (abs(auPos[i] - uPos) > 2) {
					auPos[i] = uPos;
					if (fChange)
						DBGSerial.print(", ");
					else
						fChange = true;
					DBGSerial.print(pgm_read_byte(&cPinTable[i]), DEC);
					DBGSerial.print(": ");
					DBGSerial.print(uPos, DEC);
				}
			}
		}
		if (fChange)
			DBGSerial.println();
		delay(25);
	}
	// Print out Mins and Max.
	//DBGSerial.println("    T     F     C |     C     F     T");
	static const char* apszLegs[] = {
		"RR", "RM", "RF", "LR", "LM", "LF"
	};      // Leg Order

	for (int legs = 0; legs < 6; legs++) {
		DBGSerial.printf("#define c%sCoxaMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTCOXAPIN + legs]);
		DBGSerial.printf("#define c%sCoxaMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTCOXAPIN + legs]);
		DBGSerial.printf("#define c%sFemurMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTFEMURPIN + legs]);
		DBGSerial.printf("#define c%sFemurMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTFEMURPIN + legs]);
		DBGSerial.printf("#define c%sTibiaMin1\t%d\n", apszLegs[legs], servo_mins[FIRSTTIBIAPIN + legs]);
		DBGSerial.printf("#define c%sTibiaMax1\t%d\n", apszLegs[legs], servo_maxs[FIRSTTIBIAPIN + legs]);
	}
#if 0
	DBGSerial.println("\nExit Track servos - Min/Max values");
	for (int i = 0; i < NUMSERVOS; i++) {
		if (servo_maxs[i] != servo_mins[i]) {
			DBGSerial.print(cPinTable[i], DEC);
			DBGSerial.print(" Min: ");
			DBGSerial.print(servo_mins[i], DEC);
			DBGSerial.print(" Max: ");
			DBGSerial.println(servo_maxs[i], DEC);
		}
	}
#endif
}


#endif

#ifdef OPT_FIND_SERVO_OFFSETS
//==============================================================================
//  FindServoOffsets - Find the zero points for each of our servos...
//==============================================================================
#ifndef NUMSERVOSPERLEG
#define NUMSERVOSPERLEG 3
#endif

void LSSServoDriver::FindServoOffsets()
{
	// not clean but...
	signed short asOffsets[NUMSERVOSPERLEG * CNT_LEGS];      // we have 18 servos to find/set offsets for...

	static const char* apszLegs[] = {
		"RR", "RM", "RF", "LR", "LM", "LF"
	};      // Leg Order
	static const char* apszLJoints[] = {
		" Coxa", " Femur", " Tibia", " tArs"
	};   // which joint on the leg...


	int data;
	short sSN;       // which servo number
	boolean fNew = true;  // is this a new servo to work with?
	boolean fExit = false;  // when to exit

	for (uint8_t i = 0; i < NUMSERVOS; i++) {
		asOffsets[i] = 0;
	}

	if (CheckVoltage()) {
		// Voltage is low...
		Serial.println("Low Voltage: fix or hit $ to abort");
		while (CheckVoltage()) {
			if (Serial.read() == '$')  return;
		}
	}
	// Now lets enable all servos and set them to zero point
	MakeSureServosAreOn();
	LSS::genericWrite(LSS_BroadcastID, LSS_ActionMove, 0,
	                  LSS_ActionParameterTime, 500);  // move in half second

// OK lets move all of the servos to their zero point.
	Serial.println("Find Servo Zeros.\n$-Exit, +- changes, *-change servo");
	Serial.println("    0-n Chooses a leg, C-Coxa, F-Femur, T-Tibia");
	//#define NUMSERVOS (NUMSERVOSPERLEG*CNT_LEGS)

	// Lets show some information about each of the servos.
	for (sSN = 0; sSN < NUMSERVOS; sSN++) {
		asOffsets[sSN] = 0;
		myLSS.setServoID(cPinTable[sSN]);
		Serial.print("Servo: ");
		Serial.print(apszLegs[sSN % CNT_LEGS]);
		Serial.print(apszLJoints[sSN / CNT_LEGS]);
		Serial.print("(");
		Serial.print(cPinTable[sSN], DEC);
		Serial.print(") Pos:");
		Serial.print(myLSS.getPosition(), DEC);
		Serial.print(" Origin Offset: ");
		Serial.print(myLSS.getOriginOffset(), DEC);
		Serial.print(" Angular Range: ");
		Serial.println(myLSS.getAngularRange(), DEC);

	}


	sSN = 0;
	bool data_received = false;
	while (!fExit) {
		if (fNew) {
			uint8_t servo_id = cPinTable[sSN];
			Serial.print("Servo: ");
			Serial.print(apszLegs[sSN % CNT_LEGS]);
			Serial.print(apszLJoints[sSN / CNT_LEGS]);
			Serial.print("(");
			Serial.print(servo_id, DEC);
			Serial.println(")");

			myLSS.setServoID(servo_id);
			myLSS.moveT(asOffsets[sSN], 250);
			//myLSS.move(asOffsets[sSN]);
			delay(250);
			myLSS.moveT(asOffsets[sSN] + 100, 250);
			//myLSS.move(asOffsets[sSN] + 100);
			delay(250);
			myLSS.moveT(asOffsets[sSN] - 100, 250);
			//myLSS.move(asOffsets[sSN] - 100);
			delay(250);
			myLSS.moveT(asOffsets[sSN], 250);
			//myLSS.move(asOffsets[sSN]);
			delay(250);
			fNew = false;
		}

		//get user entered data
		data = Serial.read();
		//if data received
		if (data >= '\r') {
			if (data == '\r') {
				if (!data_received) {
					// direct enter of which servo to change
					fNew = true;
					sSN++;
					if (sSN == CNT_LEGS * NUMSERVOSPERLEG)
						sSN = 0;
				}
				data_received = false;
			}
			else {
				data_received = true;
				if (data == '$')
					fExit = true; // not sure how the keypad will map so give NL, CR, LF... all implies exit

				else if ((data == '+') || (data == '-')) {
					if (data == '+')
						asOffsets[sSN] += 5;    // increment by 5us
					else
						asOffsets[sSN] -= 5;    // increment by 5us

					Serial.print("    ");
					Serial.println(asOffsets[sSN], DEC);

					myLSS.moveT(asOffsets[sSN], 100);
					//myLSS.move(asOffsets[sSN]);
				}
				else if ((data >= '0') && (data <= '5')) {
					// direct enter of which servo to change
					fNew = true;
					//sSN = (sSN % NUMSERVOSPERLEG) + (data - '0') * NUMSERVOSPERLEG;
					sSN = (sSN / CNT_LEGS) * CNT_LEGS + (data - '0');
				}
				else if ((data == 'c') || (data == 'C')) {
					fNew = true;
					//sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 0;
					sSN = sSN % CNT_LEGS;
				}
				else if ((data == 'f') || (data == 'F')) {
					fNew = true;
					//sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
					sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 1;
					sSN = CNT_LEGS + (sSN % CNT_LEGS);
				}
				else if ((data == 't') || (data == 'T')) {
					// direct enter of which servo to change
					fNew = true;
					sSN = (sSN / NUMSERVOSPERLEG) * NUMSERVOSPERLEG + 2;
					sSN = 2 * CNT_LEGS + (sSN % CNT_LEGS);
				}
				else if (data == '*') {
					// direct enter of which servo to change
					fNew = true;
					sSN++;
					if (sSN == CNT_LEGS * NUMSERVOSPERLEG)
						sSN = 0;
				}
			}
		}
	}
	Serial.print("Find Servo exit ");
	for (sSN = 0; sSN < NUMSERVOS; sSN++) {
		Serial.print("Servo: ");
		Serial.print(apszLegs[sSN / NUMSERVOSPERLEG]);
		Serial.print(apszLJoints[sSN % NUMSERVOSPERLEG]);
		Serial.println();
	}

#if 0
	Serial.print("\nSave Changes? Y/N: ");

	//get user entered data
	while (((data = Serial.read()) == -1) || ((data >= 10) && (data <= 15)))
		;

	if ((data == 'Y') || (data == 'y')) {
		// Ok they asked for the data to be saved.  We will store the data with a
		// number of servos (byte)at the start, followed by a byte for a checksum...followed by our offsets array...
		// Currently we store these values starting at EEPROM address 0. May later change...
		//

		for (sSN = 0; sSN < CNT_LEGS * NUMSERVOSPERLEG; sSN++) {
			SSCSerial.print("R");
			SSCSerial.print(32 + abSSCServoNum[sSN], DEC);
			SSCSerial.print("=");
			SSCSerial.println(asOffsetsRead[sSN] + asOffsets[sSN], DEC);
			delay(10);
		}

		// Then I need to have the SSC-32 reboot in order to use the new values.
		delay(10);    // give it some time to write stuff out.
		SSCSerial.println("GOBOOT");
		delay(5);        // Give it a little time
		SSCSerial.println("g0000");    // tell it that we are done in the boot section so go run the normall SSC stuff...
		delay(500);                // Give it some time to boot up...
	}
	else {
		void LoadServosConfig();
	}
#endif
	g_ServoDriver.FreeServos();

}
#endif  // OPT_FIND_SERVO_OFFSETS

//==============================================================================
// WakeUpRoutine - Wake up robot in a friendly way
//==============================================================================
#define DEBUG_WakeUp_Pos
void LSSServoDriver::WakeUpRoutine(void) {
	byte LegIndex;
	int CurrentCoxaPos;//was word, changed to integer to prevent since faulty reading return -1
	int CurrentFemurPos;
	int CurrentTibiaPos;
	boolean PosOK = true;

#define PosMargin 12	//we must wait if the difference between current ServoPos and IKpos is larger than this margin (12 is just over one deg in difference for MX servos)
	if (g_WakeUpState) { //Check if all servos has reached their goal position
		DBGSerial.println("Enter: LSSServoDriver::WakeUpRoutine"); DBGSerial.flush();
		for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {//for (LegIndex = CNT_LEGS / 2; LegIndex < CNT_LEGS; LegIndex++) {//Left legs

			CurrentCoxaPos = GetServoPosition(cPinTable[FIRSTCOXAPIN + LegIndex]);
			CurrentFemurPos = GetServoPosition(cPinTable[FIRSTFEMURPIN + LegIndex]);
			CurrentTibiaPos = GetServoPosition(cPinTable[FIRSTTIBIAPIN + LegIndex]);


			if ((abs((int)CurrentCoxaPos - (int)(CoxaAngle[LegIndex] )) > PosMargin) && (CurrentCoxaPos <= ServoRes) && (CurrentCoxaPos >= -ServoRes)) {
				PosOK = false;
			}
			if ((abs((int)CurrentFemurPos - (int)(FemurAngle[LegIndex] )) > PosMargin) && (CurrentFemurPos <= ServoRes) && (CurrentFemurPos >= -ServoRes)) {
				PosOK = false;
			}
			if ((abs((int)CurrentTibiaPos - (int)(TibiaAngle[LegIndex] )) > PosMargin) && (CurrentTibiaPos <= ServoRes) && (CurrentTibiaPos >= -ServoRes)) {
				PosOK = false;
			}
#ifdef DEBUG_WakeUp_Pos
			DBGSerial.print(CurrentCoxaPos, DEC);
			DBGSerial.print("-");
			DBGSerial.print((int)(CoxaAngle[LegIndex]), DEC);//must invert Right legs
			DBGSerial.print(" ");
			DBGSerial.print(CurrentFemurPos, DEC);
			DBGSerial.print("-");
			DBGSerial.print((int)(FemurAngle[LegIndex]), DEC);
			DBGSerial.print(" ");
			DBGSerial.print(CurrentTibiaPos, DEC);
			DBGSerial.print("-");
			DBGSerial.print((int)(TibiaAngle[LegIndex]), DEC);
			DBGSerial.print(" _ ");
#endif
			delay(25);
		}
		//g_InputController.AllowControllerInterrupts(true);
		DBGSerial.println(PosOK, DEC);
		if ((millis() - lWakeUpStartTime) > 6000) {
			MSound(1, 150, 1500);// Make some sound if it takes more than 6 second to get into wakeup position, something is probably wrong..
		}
		if (PosOK) { // All servos are in position, ready for turning on full torque!
			//g_InputController.AllowControllerInterrupts(false);

			g_WakeUpState = false;
		#if 0

			myLSS.setServoID(LSS_BroadcastID);
#ifdef SafetyMode
			myLSS.hold();
			delay(500); //Waiting half a second test bug bug
			LSS::genericWrite(LSS_BroadcastID, "MMD", 300);  // Reduced Torque can be in range 255 to 1023
#else
			myLSS.hold();
			delay(500); //Waiting half a second test bug bug
			LSS::genericWrite(LSS_BroadcastID, "MMD", 1023);  // Full torque
#endif
			InputController::controller()->AllowControllerInterrupts(true);    // Ok for hserial again...
			MSound(1, 80, 2000);
			g_InControlState.ForceSlowCycleWait = 2;//Get ready slowly
		#endif 
			strcpy(g_InControlState.DataPack, "Ready!");
			g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
			g_InControlState.lWhenWeLastSetDatamode = millis();

			for (LegIndex = 0; LegIndex < CNT_LEGS; LegIndex++) {
				cInitPosY[LegIndex] = cHexGroundPos;//Lower the legs to ground
			}
		}
		DBGSerial.println("Exit: LSSServoDriver::WakeUpRoutine"); DBGSerial.flush();
	}
}

//=============================================================================
// Do our own timed moves support functions.
//=============================================================================

void LSSServoDriver::TMReset() {
	tmServoCount = 0;
}

// Add a servo to the list.
uint8_t LSSServoDriver::TMAddID(uint8_t id) {
	tmServos[tmServoCount].id = id;
	tmServos[tmServoCount].target_pos = 0;
	tmServos[tmServoCount].starting_pos = 0;
	tmServoCount++;
	return tmServoCount - 1;
}

void LSSServoDriver::TMInitWithCurrentservoPositions() {
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		myLSS.setMotionControlEnabled(0);
		tmServos[servo].starting_pos = myLSS.getPosition();
		tmServos[servo].target_pos = tmServos[servo].starting_pos;
	}
}
void LSSServoDriver::TMConfigureServos() {
	int em_mode = use_servos_timed_moves ? 1 : 0;
	DBGSerial.printf("Set Servo EM=%u\n", em_mode);
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		myLSS.setMotionControlEnabled(em_mode);
	}
}

bool LSSServoDriver::TMSetTargetByID(uint8_t id, int16_t target) {
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		if (id == tmServos[servo].id) {
			TMSetTargetByIndex(servo, target);
			return true;
		}
	}
	return false;
}

void LSSServoDriver::TMSetTargetByIndex(uint8_t index, int16_t target) {
	tmServos[index].starting_pos = tmServos[index].target_pos; // set source as last target
	tmServos[index].target_pos = target;
}
void LSSServoDriver::TMSetupMove(uint32_t move_time) {
	// BUGBUG should we output all servos every cycle?
	// start off only when they move.
	tmMovetime = move_time * 1000; // convert to us
	tmCyclesLeft = (tmMovetime + tmCycleTime / 2) / tmCycleTime;
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		myLSS.setServoID(tmServos[servo].id);
		if (tmSetupServos) myLSS.setMotionControlEnabled(0);
		if (tmServos[servo].starting_pos == -1) tmServos[servo].starting_pos = myLSS.getPosition();
		tmServos[servo].pos = tmServos[servo].starting_pos;
		tmServos[servo].cycle_delta = ((tmServos[servo].target_pos - tmServos[servo].starting_pos)); // set it first to get into floating point
		tmServos[servo].cycle_delta /= tmCyclesLeft;
	}
	tmSetupServos = false;
	tmTimer = 0;

}

int  LSSServoDriver::TMStep(bool wait) {
	if (!tmCyclesLeft) return 0;

	// BUGBUG not processing wait yet... but normally
	// can set false so can return between steps to do other stuff.
	//if (!wait && ((tmCycleTime - tmTimer) > tmMinNotwaitTime)) return -1; //
	while (tmTimer < tmCycleTime) ;
	// how many cycles.
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		if (tmServos[servo].cycle_delta) {

			int cur_pos = tmServos[servo].pos;
			tmServos[servo].pos += tmServos[servo].cycle_delta;
			int next_pos = tmServos[servo].pos;
			if (tmCyclesLeft == 1) next_pos = tmServos[servo].target_pos;
			else {
				if (tmServos[servo].cycle_delta < 0) {
					if (next_pos < tmServos[servo].target_pos) next_pos = tmServos[servo].target_pos;
				} else if (next_pos > tmServos[servo].target_pos) next_pos = tmServos[servo].target_pos;
			}
			if (next_pos != cur_pos) {
				myLSS.setServoID(tmServos[servo].id);
				myLSS.move(next_pos);
				if (next_pos == tmServos[servo].target_pos) tmServos[servo].cycle_delta = 0; // servo done
			}
		}
	}
	tmCyclesLeft--;
	tmTimer -= tmCycleTime;
	return tmCyclesLeft ? 1 : 0; //
}

void LSSServoDriver::TMPrintDebugInfo() {
#ifdef DBGSerial
	DBGSerial.println("*** TM debug info");
	DBGSerial.printf("Move Time:%u Cyle time:%u cycles:%u\n", tmMovetime, tmCycleTime, tmCyclesLeft);
	DBGSerial.println("ID\t Start\t End\tCyle Delta");
	for (uint8_t servo = 0; servo < tmServoCount; servo++) {
		DBGSerial.printf("  %u\t%d\t%d\t%f\n", tmServos[servo].id, tmServos[servo].starting_pos, tmServos[servo].target_pos, tmServos[servo].cycle_delta);
	}
#endif
}




//==============================================================================
//	FindServoOffsets - Find the zero points for each of our servos...
// 		Will use the new servo function to set the actual pwm rate and see
//		how well that works...
//==============================================================================
#ifdef OPT_FIND_SERVO_OFFSETS

void FindServoOffsets()
{

}
#endif  // 
