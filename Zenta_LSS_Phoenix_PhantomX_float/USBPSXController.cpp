//#define DEBUG_COMMANDER
//TO DO:
//Updated function explanation
//Define max Strafe length, rotation angles and translation length, defined in the Hex_Cfg and a default #ifndef here?
//Maybe just send raw control values to the main code and do the mapping there or here?
//Single leg mode and dual mode
//Adjustible init positions, front and rear, extend or retract
//Auto body rotation?
//Tapping leg random, use an autofunction in the single leg mode?
//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//Date: 25-10-2009
//Programmer: Jeroen Janssen (aka Xan)
//             Kurt Eckhardt (aka KurtE) - converted to c ported to Arduino...
//
//Hardware setup: Arbotix Commander DIY version Using two 3axis joysticks with one button on top, 2 sliderpots, 16key keypad, 6 function buttons and one power/function button
// Orientation of the 6 function buttons (same as on the Arbotix Commander): L6, L5, L4 - R3, R2, R1
//
//There are two main modes that can't be combined; Walking mode and Single Leg mode
//Button R3 are used to toogle between Walk
//

//
//====================================================================
// [Include files]
#include <Arduino.h>
#include "Hex_Cfg.h"
#include "phoenix_float.h"
#include "USBPSXController.h"
#include <USBHost_t36.h>


//[CONSTANTS]
enum {
	WALKMODE = 0, TRANSLATEMODE, ROTATEMODE, //Only two modes, walk and single leg
#ifdef OPT_SINGLELEG
	SINGLELEGMODE,
#endif
	MODECNT
};
enum {
	NORM_NORM = 0, NORM_LONG, HIGH_NORM, HIGH_LONG
};

#define ARBOTIX_TO  5000        // if we don't get a valid message in this number of mills turn off

#ifndef XBeeSerial
SoftwareSerial XBeeSerial(cXBEE_IN, cXBEE_OUT);
#endif
#ifndef USER
#define USER 13
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
JoystickController joystick1(myusb);
#if defined(BLUETOOTH)
	//BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
	BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &bluet, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "Bluet", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false, false };
#else
  BluetoothController bluet(myusb);   // version assumes it already was paired

  USBDriver* drivers[] = { &hub1, &hub2, &joystick1, &hid1, &hid2, &hid3 };
  
  #define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
  const char* driver_names[CNT_DEVICES] = { "Hub1", "Hub2", "JOY1D", "HID1" , "HID2", "HID3" };
  
  bool driver_active[CNT_DEVICES] = { false, false, false };
#endif

int user_axis[64];
uint32_t _buttons_prev = 0;
uint32_t _buttons;

// Lets also look at HID Input devices
USBHIDInput* hiddrivers[] = { &joystick1 };

#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char* hid_driver_names[CNT_DEVICES] = { "Joystick1" };

bool hid_driver_active[CNT_DEVICES] = { false };
#if defined(BLUETOOTH)
	BTHIDInput* bthiddrivers[] = { &joystick1 };
	#define CNT_BTHIDDEVICES (sizeof(bthiddrivers)/sizeof(bthiddrivers[0]))
	const char* bthid_driver_names[CNT_HIDDEVICES] = { "joystick" };
	bool bthid_driver_active[CNT_HIDDEVICES] = { false };
#endif

const __FlashStringHelper* const Gait_table[] PROGMEM = { F("Wave gait"), F("Ripple gait"), F("Tripple gait"), F("Tripod gait") };//A table to hold the names
const __FlashStringHelper* const LegH_table[] PROGMEM = { F("Max leg height"), F("Med High leg height"), F("Med Low leg height"), F("Low leg height") };//A table to hold the names
#ifdef USE_ST7789
const __FlashStringHelper* const mode_names[] PROGMEM = { F("Walk"), F("Translate"), F("Rotate"), F("Single") };
#endif


// some external or forward function references.
const short cLegLiftHeight[] PROGMEM = {
	MaxLegLiftHeight, MedHighLegLiftHeight, MedLegLiftHeight, MinLegLiftHeight
};


enum {
	BUT_L1 = 0, BUT_L2, BUT_L3, BUT_R1, BUT_R2, BUT_R3,
	BUT_TRI, BUT_SQ, BUT_X, BUT_CIRC,
	BUT_PS3, BUT_SELECT, BUT_START,
	BUT_HAT_UP, BUT_HAT_DOWN, BUT_HAT_LEFT, BUT_HAT_RIGHT
};
enum { AXIS_LX, AXIS_LY, AXIS_RX, AXIS_LT, AXIS_RT, AXIS_RY };  // Order of PS3

const uint32_t USBPSXController::PS3_BTNS[] = { 0x400, 0x100, 0x2, 0x800, 0x200, 0x4,
                                                       0x1000, 0x8000, 0x4000, 0x2000,
                                                       0x10000, 0x1, 0x8,
                                                       // UP  DN    LFT   RHT
                                                       0x10, 0x40, 0x80, 0x20
                                                     };

const uint32_t USBPSXController::PS4_BTNS[] = {
	0x10, 0x40, 0x400, 0x20, 0x80, 0x800,    //L1,L2,??? , R1, R2 ?????
	0x8, 0x1, 0x2, 0x4,         //tri, square, cross, circle
	0x1000, 0x200, 0x100,       // PS, options, Share
	0x10000, 0x40000, 0x80000, 0x20000       // HAT - up, down, left, right
};
const uint32_t USBPSXController::PS4_MAP_HAT_MAP[] = {
	//0x10, 0x30, 0x20, 0x60, 0x40, 0xc0, 0x80, 0x90, 0x00 };
	0x10000, 0x30000, 0x20000, 0x60000, 0x40000, 0xC0000, 0x80000, 0x90000, 0x0
	//up,    NE,      right,   SE,      down,    SW,      left,    NW,   ???
};

 static bool joystick_ps4_bt = false;

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void USBPSXController::Init(void)
{
	// moved from phoenix_code.cpp
	SmDiv = 10; //Default SmootControl divider

	_bodyYOffset = 0;//Zenta, Start raising up after wakeuproutine
	_bodyYShift = 0;
	_bodyYpos = 0;
	_bodyZShift = 0;
	_bodyZpos = 0;
	_isRightSliderInitated = 0;
	_isLeftSliderInitated = 0;

#ifdef DBGSerial
	DBGSerial.println("USB Joystick Init: ");
#endif
	myusb.begin();

#ifdef _SPARKFUN_QWIIC_KEYPAD_ARDUINO_LIBRARY_H
	_haveKeypad = _keypad.begin(); 			//Need to pass in which bus Wire?

#ifdef DBGSerial
	if (_haveKeypad )  DBGSerial.println("Sparkfun Qwiic keypad found");
#endif
#endif


	GPSeq = 0;  // init to something...

	_controlMode = WALKMODE;
	_heightSpeedMode = NORM_NORM;
	g_InhibitMovement = true;//Do not allow body movement and walking at startup
	//    DoubleHeightOn = false;
	_doubleTravelOn = true;//Zenta want this to start with
	_bJoystickWalkMode = 0;
	_delayedWalkMode = false;
	g_InControlState.SpeedControl = 100;
	MSound(1, 250, 500);//Some sound to indicate init is done
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void USBPSXController::AllowControllerInterrupts(boolean fAllow)
{
	// We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the Commander and then
//process any commands.
//==============================================================================
#if 1 
void USBPSXController::ControlInput(void)
{
	// try the one direct from LSS_Phoenix_USB_joystick.cpp in Test stuff.
	// Make sure USB gets chance to process stuff. 
	myusb.Task();
	// check to see if the device list has changed:
	UpdateActiveDeviceInfo();
	//  processPS3MotionTimer();  - not sure yet support this yet. 

	if (joystick1.available()) {
		if (_first_joystick_message) {
			Serial.printf("*** First Joystick message %x:%x ***\n",
				joystick1.idVendor(), joystick1.idProduct());
			_first_joystick_message = false;

			const uint8_t* psz = joystick1.manufacturer();
			if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
			const uint8_t* pszProduct = joystick1.product();
			if (pszProduct && *pszProduct) Serial.printf("  product: %s\n", pszProduct);
			psz = joystick1.serialNumber();
			if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

			if (pszProduct && strncmp((const char *)pszProduct, "Wireless Controller", 19) == 0) {
				BTN_MASKS = PS4_BTNS;
				if (bluet) {
					joystick_ps4_bt = true;
					DBGSerial.println("      BlueTooth PS4");					
				} else {
					DBGSerial.println("      Wired PS4");					
				}
			}

			// lets try to reduce number of fields that update
			joystick1.axisChangeNotifyMask(0xFFFFFl);
		}

		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = false;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;

#if 0
		if (!g_InControlState.fRobotOn) {
			g_InControlState.fRobotOn = true;
			fAdjustLegPositions = true;
		}
#endif
		// [SWITCH MODES]
		_buttons = joystick1.getButtons();
#if defined(BLUETOOTH)
		if (joystick_ps4_bt || joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick_ps4_bt? joystick1.getAxis(10) : joystick1.getAxis(9);  // get hat - up/dwn buttons
#else
		if (joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick1.getAxis(9);  // get hat - up/dwn buttons
#endif
			if ((hat >= 0) && (hat < 8)) _buttons |= PS4_MAP_HAT_MAP[hat];
			BTN_MASKS = PS4_BTNS;	// should have been set earlier, but just in case...
		}
		else {
			BTN_MASKS = PS3_BTNS;
		}

#if defined(BLUETOOTH)
		if ((_buttons & BTN_MASKS[BUT_PS3]) && !(_buttons_prev & BTN_MASKS[BUT_PS3])) {
			if ((joystick1.joystickType() == JoystickController::PS3) &&
				(_buttons & (BTN_MASKS[BUT_L1] | BTN_MASKS[BUT_R1]))) {
				// PS button just pressed and select button pressed act like PS4 share like...
				// Note: you can use either R1 or L1 with the PS button, to work with Sony Move Navigation...
				Serial.print("\nPS3 Pairing Request");
				if (!_last_bdaddr[0] && !_last_bdaddr[1] && !_last_bdaddr[2] && !_last_bdaddr[3] && !_last_bdaddr[4] && !_last_bdaddr[5]) {
					Serial.println(" - failed - no Bluetooth adapter has been plugged in");
				}
				else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
					Serial.println(" - failed - PS3 device not plugged into USB");
				}
				else {
					Serial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", _last_bdaddr[0], _last_bdaddr[1], _last_bdaddr[2], _last_bdaddr[3], _last_bdaddr[4], _last_bdaddr[5]);

					if (!joystick1.PS3Pair(_last_bdaddr)) {
						Serial.println("  Pairing call Failed");
					}
					else {
						Serial.println("  Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
					}
				}
			} else {
				// Maybe lets toggle the 
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					Serial.println("Robot Power On.....");
				}
				else {
					controllerTurnRobotOff();
					Serial.println("Robot Power Off.....");
				}
			}
		}
#else
		if ((_buttons & BTN_MASKS[BUT_PS3]) && !(_buttons_prev & BTN_MASKS[BUT_PS3])) {
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
					g_WakeUpState = true;//Start the wakeup routine
					Serial.println("Robot Power On.....");
				}
				else {
					controllerTurnRobotOff();
					Serial.println("Robot Power Off.....");
				}
		}
#endif

		// Cycle through modes...
		if ((_buttons & BTN_MASKS[BUT_TRI]) && !(_buttons_prev & BTN_MASKS[BUT_TRI])) {
			if (++_controlMode >= MODECNT) {
				_controlMode = WALKMODE;    // cycled back around...
				MSound(2, 50, 2000, 50, 3000);
			}
			else {
				MSound(1, 50, 2000);
			}
#ifdef OPT_SINGLELEG      
			if (_controlMode != SINGLELEGMODE)
				g_InControlState.SelectedLeg = 255;
			else {
				g_InControlState.SelectedLeg = 5;   // Select leg 5 (LF) when we go into this mode. 
				//g_InControlState.PrevSelectedLeg = 255;
#ifdef DEBUG_SINGLELEG
				Serial.println("Single Leg Mode .....");
#endif
			}
#endif
			// If we have display display new mode
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_MODE);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print(mode_names[_controlMode]);
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
			Serial.println(mode_names[_controlMode]);

		}

		//[Common functions]
		//Switch Balance mode on/off 
		if ((_buttons & BTN_MASKS[BUT_SQ]) && !(_buttons_prev & BTN_MASKS[BUT_SQ])) {
			g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
			if (g_InControlState.BalanceMode) {
				MSound(1, 250, 1500);
				Serial.println("Balance Mode: On ........");
			}
			else {
				MSound(2, 100, 2000, 50, 4000);
				Serial.println("Balance Mode: Off ........");
			}

		}

		//Stand up, sit down  
		if ((_buttons & BTN_MASKS[BUT_HAT_DOWN]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_DOWN])) {
			_bodyYOffset = 0;
			Serial.println("Sitting ........");
		}
		if ((_buttons & BTN_MASKS[BUT_HAT_UP]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_UP])) {
			if (_bodyYOffset < 35) _bodyYOffset = 35;
			else _bodyYOffset += 10;
			if (_bodyYOffset > MAX_BODY_Y) _bodyYOffset = MAX_BODY_Y;
			fAdjustLegPositions = true;
			_fDynamicLegXZLength = false;
			Serial.println("Ready for Action ........");
		}
		// add some speed control
		if ((_buttons & BTN_MASKS[BUT_HAT_RIGHT]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_RIGHT])) {
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
		}
		if ((_buttons & BTN_MASKS[BUT_HAT_LEFT]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_LEFT])) {
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
		}


		// We will use L1 with the Right joystick to control both body offset as well as Speed...
		// We move each pass through this by a percentage of how far we are from center in each direction
		// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
		// which give an idea, but only for those whoes robot has a speaker
		int lx = joystick1.getAxis(AXIS_LX) - 127;
		int ly = joystick1.getAxis(AXIS_LY) - 127;
		int rx = joystick1.getAxis(AXIS_RX) - 127;
		int ry = joystick1.getAxis(AXIS_RY) - 127;
		if (_fDebugJoystick) {
			Serial.printf("(%d)BTNS: %x LX: %d, LY: %d, RX: %d, RY: %d LT: %d RT: %d\r\n", joystick1.joystickType(), _buttons,
				lx, ly, rx, ry, joystick1.getAxis(AXIS_LT), joystick1.getAxis(AXIS_RT));
		}

		if (_buttons & BTN_MASKS[BUT_L1] && !(_buttons_prev & BTN_MASKS[BUT_L1])) {
			Serial.println("Adjusting up/down (Ry) /speed (Rx) ........");
			// raise or lower the robot on the joystick up /down
			// Maybe should have Min/Max
			int delta = ry / 25;
			if (delta) {
				_bodyYOffset = max(min(_bodyYOffset + delta, MAX_BODY_Y), 0);
				fAdjustLegPositions = true;
			}

			// Also use right Horizontal to manually adjust the initial leg positions.
			sLegInitXZAdjust = lx / 10;        // play with this.
			sLegInitAngleAdjust = ly / 8;
			lx = 0;
			ly = 0;

			// Likewise for Speed control
			delta = rx / 16;   // 
			if ((delta < 0) && g_InControlState.SpeedControl) {
				if ((word)(-delta) < g_InControlState.SpeedControl)
					g_InControlState.SpeedControl += delta;
				else
					g_InControlState.SpeedControl = 0;
				MSound(1, 50, 1000 + g_InControlState.SpeedControl);
			}
			if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
				g_InControlState.SpeedControl += delta;
				if (g_InControlState.SpeedControl > 2000)
					g_InControlState.SpeedControl = 2000;
				MSound(1, 50, 1000 + g_InControlState.SpeedControl);
			}

			rx = 0; // don't walk when adjusting the speed here...
		}

#ifdef DBGSerial
		if ((_buttons & BTN_MASKS[BUT_X]) && !(_buttons_prev & BTN_MASKS[BUT_X])) {
			MSound(1, 50, 2000);
			_fDebugJoystick = !_fDebugJoystick;
		}
		if ((_buttons & BTN_MASKS[BUT_L3]) && !(_buttons_prev & BTN_MASKS[BUT_L3])) {
			MSound(1, 50, 2000);
#if defined(TEENSY_DEBUG_H)
			if (debug.isGDBConnected()) {
				DBGSerial.println(F("Trying to break in to GDB"));
				// lets try it through setting breakpoint at loop
				debug.setBreakpoint((void*)&loop);
				//halt();
			}
			else
#endif
			{
				g_fDebugOutput = !g_fDebugOutput;
				if (g_fDebugOutput) DBGSerial.println(F("Debug is on"));
				else DBGSerial.println(F("Debug is off"));
			}

		}
#endif    
		//[Walk functions]
		if (_controlMode == WALKMODE) {
			//Switch Gaits
			if (((_buttons & BTN_MASKS[BUT_SELECT]) && !(_buttons_prev & BTN_MASKS[BUT_SELECT]))
				&& abs(g_InControlState.TravelLength.x) < cTravelDeadZone //No movement
				&& abs(g_InControlState.TravelLength.z) < cTravelDeadZone
				&& abs(g_InControlState.TravelLength.y * 2) < cTravelDeadZone) {
				g_InControlState.GaitType = g_InControlState.GaitType + 1;                    // Go to the next gait...
				if (g_InControlState.GaitType < NumOfGaits) {                 // Make sure we did not exceed number of gaits...
					MSound(1, 50, 2000);
				}
				else {
					MSound(2, 50, 2000, 50, 2250);
					g_InControlState.GaitType = 0;
				}
				strcpy(g_InControlState.DataPack, (const char *)Gait_table[g_InControlState.GaitType]);
				g_InControlState.DataMode = 1;
				g_InControlState.lWhenWeLastSetDatamode = millis();
				Serial.printf("Gait Selected: %s\n", (const char *)Gait_table[g_InControlState.GaitType]) ;
				//GaitSelect();
			}
 
			//Double leg lift height
			if ((_buttons & BTN_MASKS[BUT_R2]) && !(_buttons_prev & BTN_MASKS[BUT_R2])) {
				MSound(1, 50, 2000);
				_heightSpeedMode = (_heightSpeedMode + 1) & 0x3; // wrap around mode
				_doubleTravelOn = _heightSpeedMode & 0x1;
				if (_doubleTravelOn) Serial.print("Double Travel On - ");
				else Serial.print("Double Travel Off - ");

				if (_heightSpeedMode & 0x2) {
					g_InControlState.LegLiftHeight = 80;
					Serial.println("Double Leg Height Selected .....");
				} else {
					g_InControlState.LegLiftHeight = 50;
					Serial.println("Normal Leg Height Selected .....");
				}
			}

			// Switch between Walk method 1 && Walk method 2
			if ((_buttons & BTN_MASKS[BUT_CIRC]) && !(_buttons_prev & BTN_MASKS[BUT_CIRC])) {
#ifdef cTurretRotPin
				if ((++_bJoystickWalkMode) > 2)
#else
				if ((++_bJoystickWalkMode) > 1)
#endif 
					_bJoystickWalkMode = 0;
				MSound(1, 50, 2000 + _bJoystickWalkMode * 250);
				Serial.printf("Walkmethod %d Selected ........\n", _bJoystickWalkMode );
			}

			//Walking
			switch (_bJoystickWalkMode) {
			case 0:
				g_InControlState.TravelLength.x = -lx;
				g_InControlState.TravelLength.z = -ly;
				g_InControlState.TravelLength.y = -(rx) / 4; //Right Stick Left/Right 
				break;
			case 1:
				g_InControlState.TravelLength.z = (joystick1.getAxis(AXIS_RY)); //Right Stick Up/Down  
				g_InControlState.TravelLength.y = -(rx) / 4; //Right Stick Left/Right 
				break;
#ifdef cTurretRotPin
			case 2:
				g_InControlState.TravelLength.x = -lx;
				g_InControlState.TravelLength.z = -ly;

				// Will use Right now stick to control turret.
				g_InControlState.TurretRotAngle1 = max(min(g_InControlState.TurretRotAngle1 + rx / 5, cTurretRotMax1), cTurretRotMin1);      // Rotation of turret in 10ths of degree
				g_InControlState.TurretTiltAngle1 = max(min(g_InControlState.TurretTiltAngle1 + joystick1.getAxis(AXIS_RY) / 5, cTurretTiltMax1), cTurretTiltMin1);  // tilt of turret in 10ths of degree
#endif
			}

			if (!_doubleTravelOn) {  //(Double travel length)
				g_InControlState.TravelLength.x = g_InControlState.TravelLength.x / 2;
				g_InControlState.TravelLength.z = g_InControlState.TravelLength.z / 2;
			}

		}

		//[Translate functions]
		_bodyYShift = 0;
		if (_controlMode == TRANSLATEMODE) {
			g_InControlState.BodyPos.x = SmoothControl(((lx) * 2 / 3), g_InControlState.BodyPos.x, SmDiv);
			g_InControlState.BodyPos.z = SmoothControl(((ly) * 2 / 3), g_InControlState.BodyPos.z, SmDiv);
			g_InControlState.BodyRot1.y = SmoothControl(((rx) * 2), g_InControlState.BodyRot1.y, SmDiv);

			//      g_InControlState.BodyPos.x = (lx)/2;
			//      g_InControlState.BodyPos.z = -(ly)/3;
			//      g_InControlState.BodyRot1.y = (rx)*2;
			_bodyYShift = (-(joystick1.getAxis(AXIS_RY)-127) / 2); //Zenta should be joystick1.getAxis(AXIS_RY) - 127
		}

		//[Rotate functions]
		if (_controlMode == ROTATEMODE) {
			g_InControlState.BodyRot1.x = (ly);
			g_InControlState.BodyRot1.y = (rx) * 2;
			g_InControlState.BodyRot1.z = (lx);
			_bodyYShift = (-(joystick1.getAxis(AXIS_RY)-127) / 2); //Zenta Should be -127 or just replace with ry
		}
		//[Single leg functions]
#ifdef OPT_SINGLELEG      
		if (_controlMode == SINGLELEGMODE) {
			//Switch leg for single leg control
			if ((_buttons & BTN_MASKS[BUT_SELECT]) && !(_buttons_prev & BTN_MASKS[BUT_SELECT])) {
				MSound(1, 50, 2000);
				if (g_InControlState.SelectedLeg < (CNT_LEGS - 1)) {
					g_InControlState.SelectedLeg = g_InControlState.SelectedLeg + 1;
					Serial.printf("Leg #%d Selected .....\n", g_InControlState.SelectedLeg);
				} else {
					g_InControlState.SelectedLeg = 0;
				}
			}

#if 0
			g_InControlState.SLLeg.x = (signed char)((int)((int)lx + 128) / 2); //Left Stick Right/Left
			g_InControlState.SLLeg.y = (signed char)((int)((int)joystick1.getAxis(AXIS_RY) + 128) / 10); //Right Stick Up/Down
			g_InControlState.SLLeg.z = (signed char)((int)((int)ly + 128) / 2); //Left Stick Up/Down
#else
			// BUGBUG:: Need to figure out a decent range for these values... 
			g_InControlState.SLLeg.x = lx; //Left Stick Right/Left
			g_InControlState.SLLeg.y = joystick1.getAxis(AXIS_RY) / 3 - 20; //Right Stick Up/Down
			g_InControlState.SLLeg.z = ly; //Left Stick Up/Down
#endif
#ifdef DEBUG_SINGLELEG
			Serial.print(g_InControlState.SLLeg.x, DEC);
			Serial.print(",");
			Serial.print(g_InControlState.SLLeg.y, DEC);
			Serial.print(",");
			Serial.println(g_InControlState.SLLeg.z, DEC);
#endif
			// Hold single leg in place
			if ((_buttons & BTN_MASKS[BUT_R1]) && !(_buttons_prev & BTN_MASKS[BUT_R1])) {
				MSound(1, 50, 2000);
				g_InControlState.fSLHold = !g_InControlState.fSLHold;
				Serial.println("Holding selected leg in position ...........");
			}
		}
#endif

		//Calculate walking time delay
		g_InControlState.InputTimeDelay = 128 - max(max(abs(lx), abs(ly)), abs(rx));

		//Calculate g_InControlState.BodyPos.y
		g_InControlState.BodyPos.y = max(_bodyYOffset + _bodyYShift, 0);

		if (sLegInitXZAdjust || sLegInitAngleAdjust) {
			// User asked for manual leg adjustment - only do when we have finished any previous adjustment

			if (!g_InControlState.ForceGaitStepCnt) {
				if (sLegInitXZAdjust)
					_fDynamicLegXZLength = true;

				sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
				// Handle maybe change angles...
				if (sLegInitAngleAdjust)
					RotateLegInitAngles(sLegInitAngleAdjust);

				// Give system time to process previous calls
				AdjustLegPositions(sLegInitXZAdjust);
			}
		}

		if (fAdjustLegPositions && !_fDynamicLegXZLength)
			AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

		  // Save away the buttons state as to not process the same press twice.
		_buttons_prev = _buttons;
		_ulLastMsgTime = millis();
	}
	else {
		// We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
		if (g_InControlState.fRobotOn) {
			if (!joystick1)
				controllerTurnRobotOff();
		}
	}
}

#else
void USBPSXController::ControlInput(void)
{
	// See if we have a new command available...
	// Make sure USB gets chance to process stuff.
	myusb.Task();
	// check to see if the device list has changed:
	UpdateActiveDeviceInfo();
	//  processPS3MotionTimer();  - not sure yet support this yet.


	if (joystick1.available()) {
		// If we receive a valid message than turn robot on...
		boolean fAdjustLegPositions = false;
		short sLegInitXZAdjust = 0;
		short sLegInitAngleAdjust = 0;

		if (_first_joystick_message) {
			DBGSerial.printf("*** First Joystick message %x:%x ***\n",
			                 joystick1.idVendor(), joystick1.idProduct());
			_first_joystick_message = false;

			const uint8_t* psz = joystick1.manufacturer();
			if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
			psz = joystick1.product();
			if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
			psz = joystick1.serialNumber();
			if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

			// lets try to reduce number of fields that update
			//joystick1.axisChangeNotifyMask(0xFFFFFl);
		}
		_buttons = joystick1.getButtons();
		// [SWITCH MODES]
		// We will use L1 button with the Right joystick to control both body offset as well as Speed...
		// We move each pass through this by a percentage of how far we are from center in each direction
		// We get feedback with height by seeing the robot move up and down.  For Speed, I put in sounds
		// which give an idea, but only for those whoes robot has a speaker
		int lx = (joystick1.getAxis(AXIS_LX) - 127);
		int ly = (joystick1.getAxis(AXIS_LY) - 127);
		int rx = (joystick1.getAxis(AXIS_RX) - 127);
		int ry = (joystick1.getAxis(AXIS_RY) - 127);
#ifdef DBGSerial
		if (_fDebugJoystick) {
			DBGSerial.printf("(%d)BTNS: %x LX: %d, LY: %d, RX: %d, RY: %d LT: %d RT: %d\r\n", joystick1.joystickType(), _buttons,
			                 lx, ly, rx, ry, joystick1.getAxis(AXIS_LT), joystick1.getAxis(AXIS_RT));
		}
#endif

#if defined(BLUETOOTH)
		if (joystick_ps4_bt || joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick_ps4_bt? joystick1.getAxis(10) : joystick1.getAxis(9);  // get hat - up/dwn buttons
#else
		if (joystick1.joystickType() == JoystickController::PS4) {
			int hat = joystick1.getAxis(9);  // get hat - up/dwn buttons
#endif
			if ((hat >= 0) && (hat < 8)) _buttons |= PS4_MAP_HAT_MAP[hat];
			BTN_MASKS = PS4_BTNS;	// should have been set earlier, but just in case...
		}
		else {
			BTN_MASKS = PS3_BTNS;
		}		
		
		
		
		
		

#ifdef _SPARKFUN_QWIIC_KEYPAD_ARDUINO_LIBRARY_H
	if (_haveKeypad) {
		_keypad.updateFIFO();
		_keypad_button = _keypad.getButton();
	}  			//Need to pass in which bus Wire?
#endif

#if defined(BLUETOOTH)
		if ((_buttons & BTN_MASKS[BUT_PS3]) && !(_buttons_prev & BTN_MASKS[BUT_PS3])){
			if ((joystick1.joystickType() == JoystickController::PS3) &&
			    (_buttons & (BTN_MASKS[BUT_L1] | BTN_MASKS[BUT_R1]))) {
				// PS button just pressed and select button pressed act like PS4 share like...
				// Note: you can use either R1 or L1 with the PS button, to work with Sony Move Navigation...
				DBGSerial.print("\nPS3 Pairing Request");
				if (!_last_bdaddr[0] && !_last_bdaddr[1] && !_last_bdaddr[2] && !_last_bdaddr[3] && !_last_bdaddr[4] && !_last_bdaddr[5]) {
					DBGSerial.println(" - failed - no Bluetooth adapter has been plugged in");
				}
				else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
					DBGSerial.println(" - failed - PS3 device not plugged into USB");
				}
				else {
					DBGSerial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", _last_bdaddr[0], _last_bdaddr[1],
					                 _last_bdaddr[2], _last_bdaddr[3], _last_bdaddr[4], _last_bdaddr[5]);

					if (!joystick1.PS3Pair(_last_bdaddr)) {
						DBGSerial.println("  Pairing call Failed");
					}	else {
						DBGSerial.println("  Pairing complete (I hope), make sure Bluetooth adapter is plugged in and try PS3 without USB");
					}
				}
			}
			else {
				// Maybe lets toggle the
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					g_WakeUpState = true;//Start the wakeup routine

					//delay(10000);//Testing a bug that occour after powerup. Robot turns on and of and then on again. After programming first time it work fine. Then bug start after powerup
					g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
				}
				else {
					controllerTurnRobotOff();
				}
			}
		}
#else
		if ((_buttons & BTN_MASKS[BUT_PS3]) && !(_buttons_prev & BTN_MASKS[BUT_PS3])) {
				if (!g_InControlState.fRobotOn) {
					g_InControlState.fRobotOn = true;
					fAdjustLegPositions = true;
					g_WakeUpState = true;//Start the wakeup routine

					//delay(10000);//Testing a bug that occour after powerup. Robot turns on and of and then on again. After programming first time it work fine. Then bug start after powerup
					g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..
					Serial.println("Robot Power On.....");
				}
				else {
					controllerTurnRobotOff();
					Serial.println("Robot Power Off.....");
				}
		}

#endif

		if (!g_WakeUpState) {	//Don't take care of controller inputs until the WakeUpState is over (false)

			if ((_buttons & BTN_MASKS[BUT_X]))  {     //Was L3
				MSound(1, 50, 2000);
				_fDebugJoystick = !_fDebugJoystick;
			}

			// Cycle through modes...
			if ((_buttons & BTN_MASKS[BUT_TRI]) && !(_buttons_prev & BTN_MASKS[BUT_TRI])) {
				if (++_controlMode >= MODECNT) {
					_controlMode = WALKMODE;    // cycled back around...
					MSound(2, 50, 2000, 50, 3000);
				}
				else {
					MSound(1, 50, 2000);
				}
				if (_controlMode == WALKMODE) {
					strcpy(g_InControlState.DataPack, "Crawling Mode");
				}
#ifdef OPT_SINGLELEG
				if (_controlMode == SINGLELEGMODE) {
					g_InControlState.SelectedLeg = 5;//Zenta made the front right as default at start
					strcpy(g_InControlState.DataPack, "Single LT=Hld L6=Tgl");
				}
#endif
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
				
			// If we have display display new mode
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_MODE);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print(mode_names[_controlMode]);
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
			}


			//Stand up, sit down
			//if (ButtonPressed(BUT_PS3)) {
/*
			if ((_buttons & BTN_MASKS[BUT_HAT_DOWN]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_DOWN])) {	
				if (_bodyYOffset > 0) {
					_bodyYOffset = 0;
					g_InhibitMovement = true;//Do not allow body movement and walking
					strcpy(g_InControlState.DataPack, "Resting position");
				}
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..

				fAdjustLegPositions = false;//Zenta setting this to false removes a bug
				_fDynamicLegXZLength = false;
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_PWR);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print("Sitting...");
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
			}
			
				
			if ((_buttons & BTN_MASKS[BUT_HAT_UP]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_UP])) {
				if (_bodyYOffset < 35) {
					_bodyYOffset = 35;//Zenta a little higher for avoiding the out of range issue on a symmetric MKI PhanomX
					g_InhibitMovement = false; //Allow body movement and walking
					strcpy(g_InControlState.DataPack, "Ready for action!");
				}
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..

				fAdjustLegPositions = false;//Zenta setting this to false removes a bug
				_fDynamicLegXZLength = false;
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_PWR);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print("Standing...");
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
			}
*/

			//Stand up, sit down
			if ((_buttons & BTN_MASKS[BUT_HAT_DOWN])) {
				if (_bodyYOffset > 0) {
					_bodyYOffset = 0;
					g_InhibitMovement = true;//Do not allow body movement and walking
					strcpy(g_InControlState.DataPack, "Resting position");
				}
				else {
					_bodyYOffset = 80;//Zenta a little higher for avoiding the out of range issue on a symmetric MKI PhanomX
					g_InhibitMovement = false; //Allow body movement and walking
					strcpy(g_InControlState.DataPack, "Ready for action!");
				}
				g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
				g_InControlState.lWhenWeLastSetDatamode = millis();
				g_InControlState.ForceSlowCycleWait = 2;//Do this action slowly..

				fAdjustLegPositions = false;//Zenta setting this to false removes a bug
				_fDynamicLegXZLength = false;
			}

			if (_buttons & BTN_MASKS[BUT_L1]) {
#ifdef USE_ST7789
			tft.setCursor(0, TFT_Y_PWR);
			tft.setTextSize(2);
			tft.setTextColor(ST77XX_WHITE, ST77XX_RED);
			tft.print("Adjust Position/Spd...");
			tft.fillRect(tft.getCursorX(), tft.getCursorY(), tft.width(), 15, ST77XX_RED);
#endif
				int delta = ry / 25;
				if (delta) {
					_bodyYOffset = max(min(_bodyYOffset + delta, MAX_BODY_Y), 0);
					fAdjustLegPositions = true;
				}

				// Also use right Horizontal to manually adjust the initial leg positions.
				sLegInitXZAdjust = lx / 10;        // play with this.
				sLegInitAngleAdjust = ly / 8;
				lx = 0;
				ly = 0;

				// Likewise for Speed control
				delta = rx / 16;   //
				if ((delta < 0) && g_InControlState.SpeedControl) {
					if ((word)(-delta) < g_InControlState.SpeedControl)
						g_InControlState.SpeedControl += delta;
					else
						g_InControlState.SpeedControl = 0;
					MSound(1, 50, 1000 + g_InControlState.SpeedControl);
				}
				if ((delta > 0) && (g_InControlState.SpeedControl < 2000)) {
					g_InControlState.SpeedControl += delta;
					if (g_InControlState.SpeedControl > 2000)
						g_InControlState.SpeedControl = 2000;
					MSound(1, 50, 1000 + g_InControlState.SpeedControl);
				}

				rx = 0; // don't walk when adjusting the speed here...

			}

#ifdef OPT_SINGLELEG
			//Common control functions for both walking and single leg
			if ((_controlMode == WALKMODE) || (_controlMode == SINGLELEGMODE)) {
#else
			if (_controlMode == WALKMODE) {
#endif
				//Check keypad inputs:
				bool gait_changed = false;
				if ((_buttons & BTN_MASKS[BUT_HAT_LEFT])) {
					g_InControlState.GaitType++;                    			  // Go to the next gait...
					if (g_InControlState.GaitType < NumOfGaits) {                 // Make sure we did not exceed number of gaits...
						MSound(1, 50, 2000);
					} else {
						MSound(2, 50, 2000, 50, 2250);
						g_InControlState.GaitType = 0;
					}
					gait_changed = true;
					Serial.printf("Go to next gait: %d\n", g_InControlState.GaitType );
				}
				if ((_keypad_button >= '1') && (_keypad_button <= '4')) {
					g_InControlState.GaitType = _keypad_button - '1';
					gait_changed = true;
					MSound(1, 50, 2000);
					Serial.printf("Gait Type #: %d\n", g_InControlState.GaitType );
				}
				if (gait_changed) {
					//strcpy_P(g_InControlState.DataPack, (char*)pgm_read_word(&(Gait_table[Index])));
					strcpy(g_InControlState.DataPack, (const char *)Gait_table[g_InControlState.GaitType]);
					g_InControlState.DataMode = 1;
					g_InControlState.lWhenWeLastSetDatamode = millis();
					Serial.printf("Gait Selected: %s\n", (const char *)Gait_table[g_InControlState.GaitType]) ;
				}

				// Was 7 on Zentas....
				if (_keypad_button == '0') {
					if (SmDiv > 20) {
						SmDiv = 1;
						MSound(1, 50, 1000);
						strcpy(g_InControlState.DataPack, "Raw and fast control");
						Serial.printf("Gait Control: Raw and fast control\n");
					} else {
						SmDiv *= 3;
						SmDiv += 7;
						MSound(1, 50, 1500 + SmDiv * 20);
						strcpy(g_InControlState.DataPack, "Smooth control");
						if (SmDiv > 20) strcpy(g_InControlState.DataPack, "Super Smooth ctrl!");
						Serial.printf("Gait Control: Smooth control\n");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				// Was A-D on zentas...
				if ((_keypad_button >= '5') && (_keypad_button <= '8')) {
					int8_t Index = _keypad_button - '5';
					g_InControlState.LegLiftHeight = pgm_read_word(&cLegLiftHeight[Index]);//Key A = MaxLegLiftHeight
					strcpy(g_InControlState.DataPack, (char*)LegH_table[Index]);
					MSound(1, 50, 2000);
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				if (_keypad_button == '#') {
					if (g_InControlState.BodyRotOffset.y == 0) {
						g_InControlState.BodyRotOffset.y = 200;
						strcpy(g_InControlState.DataPack, "YRotation offset =20");
						Serial.printf("YRotation offset =20\n");
					}
					else {
						g_InControlState.BodyRotOffset.y = 0;
						strcpy(g_InControlState.DataPack, "YRotation offset = 0");
						Serial.printf("YRotation offset = 0\n");

					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
					MSound(1, 50, 2000);
				}

				//Switch between absolute and relative body translation and rotation. Using R1
				if ((_buttons & BTN_MASKS[BUT_HAT_RIGHT]) && !(_buttons_prev & BTN_MASKS[BUT_HAT_RIGHT]))  {
				}
				//Switch between two balance methods
				if ((_buttons & BTN_MASKS[BUT_CIRC]) && !(_buttons_prev & BTN_MASKS[BUT_CIRC]))  {
					g_InControlState.BalanceMode++;
					if (g_InControlState.BalanceMode < 2) {//toogle between two modes
						MSound(1, 250, 1500);
						strcpy(g_InControlState.DataPack, "Balance Mode ON");
					}
					else {
						g_InControlState.BalanceMode = 0;
						MSound(2, 100, 2000, 50, 4000);
						strcpy(g_InControlState.DataPack, "Balance Mode OFF");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				{	//Default body rotation
					g_InControlState.BodyRot1.x = SmoothControl((ry) * 2, g_InControlState.BodyRot1.x, SmDiv);//g_InControlState.BodyRot1.x = (command.rightV) * 2;//Zenta *2
					//g_InControlState.BodyRot1.y = SmoothControl((command.rightT) * 3, g_InControlState.BodyRot1.y, SmDiv);//g_InControlState.BodyRot1.y = (command.rightT) * 2;
					g_InControlState.BodyRot1.z = SmoothControl((-rx) * 2, g_InControlState.BodyRot1.z, SmDiv);//g_InControlState.BodyRot1.z = (-command.rightH) * 2;//Zenta, *2
				}
			}//Common functions end

			//[Walk functions]
			if (_controlMode == WALKMODE) {


				//Toogle normal start walking and delayed walk. Look around, then walk effect.
				//Toogle dampen down speed. Might run this permanently?
				if ((_buttons & BTN_MASKS[BUT_SQ]) && !(_buttons_prev & BTN_MASKS[BUT_SQ])) {
					g_InControlState.DampDwnSpeed = !g_InControlState.DampDwnSpeed;
					if (g_InControlState.DampDwnSpeed) {
						MSound(1, 250, 1500);
						strcpy(g_InControlState.DataPack, "Damping Down ON");
					}
					else {
						MSound(2, 100, 2000, 50, 4000);
						strcpy(g_InControlState.DataPack, "Damping Down OFF");
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}


				// Switch between Walking and Body translation using Left Top joystick button
				{	//Default walking mode:
#ifdef MXPhoenix


					g_InControlState.TravelLength.x = (float)((int)command.leftH);// *5 / 7; //Left Stick Right/Left about +/- 90mm
					g_InControlState.TravelLength.z = (float)(-(int)command.leftV);// *5 / 7; //Left Stick Up/Down about +/- 90mm
					g_InControlState.TravelLength.y = (float)(-command.leftT) / 4;// / 3; //Left Stick Top Pot /5
#else
					g_InControlState.TravelLength.x = (float)((int)lx) * 5 / 7; //Left Stick Right/Left about +/- 90mm
					g_InControlState.TravelLength.z = (float)(-(int)ly) * 5 / 7;//Left Stick Up/Down about +/- 90mm
					//g_InControlState.TravelLength.y = (float)(-command.leftT) / 5;//Left Stick Top Pot /5
#endif
					//Calculate walking time delay
					g_InControlState.InputTimeDelay = 128 - max(abs(lx), abs(ly));
				}
			}


			//[Single leg functions]
#ifdef OPT_SINGLELEG
			if (_controlMode == SINGLELEGMODE) {
				//Switch leg for single leg control
				if ((_buttons & BTN_MASKS[BUT_R1]) && !(_buttons_prev & BTN_MASKS[BUT_R1]))  {

					if (g_InControlState.SelectedLeg == 5) { //Only toogle between the two front legs
						g_InControlState.SelectedLeg = 2;//Right Leg
						strcpy(g_InControlState.DataPack, "Right Single Leg");
						MSound(1, 50, 2000);
					}
					else {
						g_InControlState.SelectedLeg = 5;//Left Leg
						strcpy(g_InControlState.DataPack, "Left Single Leg");
						MSound(2, 50, 2000, 50, 2250);
					}
					g_InControlState.DataMode = 1;//We want to send a text message to the remote when changing state
					g_InControlState.lWhenWeLastSetDatamode = millis();
				}
				//Zenta, Fixed. replaced byte with float, also increased range

				if ((ly >= 0) || (g_InControlState.SLLeg.y <= 0)) { //Only allow leg to move while it is lifted
					if (ly < 0) ly = 0;//Not allowing negative value
#ifdef MXPhoenix
					g_InControlState.SLLeg.y = SmoothControl((-(int)ly) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)lx), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-ry) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#else
					g_InControlState.SLLeg.y = SmoothControl((-(int)ly) * 3, g_InControlState.SLLeg.y, SmDiv); //Using left vertical for leg lifting instead, increase range even more in balancemode
					//g_InControlState.SLLeg.x = (float)((int)command.leftH); //NOT using X control anymore, Yawrot is better for the front legs
					g_InControlState.SLLeg.z = SmoothControl((-(int)lx), g_InControlState.SLLeg.z, SmDiv); //Left Stick Up/Down
					g_InControlState.SLyawRot = SmoothControl((-(int)ry) / 4, g_InControlState.SLyawRot, SmDiv);//WIP test!!!!
#endif
				}
				// Hold single leg in place
				if ((_buttons & BTN_MASKS[BUT_R2]) && !(_buttons_prev & BTN_MASKS[BUT_R2])) {
					MSound(1, 50, 2000);
					if (!g_InControlState.fSLHold && !_wantToEndSLHold) {
						_prevYposSLHold = g_InControlState.SLLeg.y; //Save the Ypos when holding it
						g_InControlState.fSLHold = true;
					}
					else {
						_wantToEndSLHold = true; //We want to change state
					}
				}
				if (_wantToEndSLHold && (g_InControlState.SLLeg.y <= _prevYposSLHold)) { //make sure the joystick is in right position before
					g_InControlState.fSLHold = false;
					_wantToEndSLHold = false;
				}


			}
#endif



			//Calculate g_InControlState.BodyPos.y
			g_InControlState.BodyPos.y = max(min(_bodyYOffset + _bodyYShift + _bodyYpos, MAX_BODY_Y), 0);//Make sure we don't get beyond the limits, should probably do more of that..
			g_InControlState.BodyPos.z = max(min(_bodyZShift + _bodyZpos, MAX_BODY_Z), MIN_BODY_Z);
			//DBGSerial.print("BPY:");
			//DBGSerial.println((int)g_InControlState.BodyPos.y, DEC);

			if (sLegInitXZAdjust || sLegInitAngleAdjust) {
				// User asked for manual leg adjustment - only do when we have finished any previous adjustment

				if (!g_InControlState.ForceGaitStepCnt) {
					if (sLegInitXZAdjust)
						_fDynamicLegXZLength = true;

					sLegInitXZAdjust += GetLegsXZLength();  // Add on current length to our adjustment...
					// Handle maybe change angles...
					if (sLegInitAngleAdjust)
						RotateLegInitAngles(sLegInitAngleAdjust);

					// Give system time to process previous calls
					AdjustLegPositions(sLegInitXZAdjust);
				}
			}

			if (fAdjustLegPositions && !_fDynamicLegXZLength)
				AdjustLegPositionsToBodyHeight();    // Put main workings into main program file

			// Save away the buttons state as to not process the same press twice.
			_buttons_prev = _buttons;
		}
		_ulLastMsgTime = millis();
		_buttons_prev = _buttons;
	}
	else {
		if (!g_WakeUpState) { //At the moment we can't turn off robot during the WakeUpState, is that a problem?
			// We did not receive a valid packet.  check for a timeout to see if we should turn robot off...
			if (g_InControlState.fRobotOn) {
				if ((millis() - _ulLastMsgTime) > ARBOTIX_TO) {
					controllerTurnRobotOff();
					DBGSerial.println("Turning OFF!");//bug we should not be here straight after wakeup!
					//For some reason I've a bug that make the robot turn off during wakeup BUG BUG
				}
			}
		}
	}
}
#endif

//==============================================================================
// CommanderTurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void USBPSXController::controllerTurnRobotOff(void)
{
	//Turn off
	g_InControlState.BodyPos.x = 0;
	g_InControlState.BodyPos.y = 0;
	g_InControlState.BodyPos.z = 0;
	g_InControlState.BodyRot1.x = 0;
	g_InControlState.BodyRot1.y = 0;
	g_InControlState.BodyRot1.z = 0;
	g_InControlState.TravelLength.x = 0;
	g_InControlState.TravelLength.z = 0;
	g_InControlState.TravelLength.y = 0;
	_bodyYOffset = 0;
	_bodyYShift = 0;
#ifdef OPT_SINGLELEG
	g_InControlState.SelectedLeg = 255;
#endif
	g_InControlState.fRobotOn = 0;

#ifdef cTurretRotPin
	g_InControlState.TurretRotAngle1 = cTurretRotInit;      // Rotation of turrent in 10ths of degree
	g_InControlState.TurretTiltAngle1 = cTurretTiltInit;    // the tile for the turret
#endif

	_fDynamicLegXZLength = false; // also make sure the robot is back in normal leg init mode...
}
//=============================================================================
// UpdateActiveDeviceInfo
//=============================================================================
void USBPSXController::UpdateActiveDeviceInfo() {
	for (uint8_t i = 0; i < CNT_DEVICES; i++) {
		if (*drivers[i] != driver_active[i]) {
			if (driver_active[i]) {
				DBGSerial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
				driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
				driver_active[i] = true;

				const uint8_t* psz = drivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = drivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = drivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				if (drivers[i] == &bluet) {
					const uint8_t* bdaddr = bluet.myBDAddr();
					// remember it...
					DBGSerial.printf("  BDADDR: %x:%x:%x:%x:%x:%x\n", bdaddr[0], bdaddr[1], bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
					for (uint8_t i = 0; i < 6; i++) _last_bdaddr[i] = bdaddr[i];
				}
			}
		}
	}

	for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
		if (*hiddrivers[i] != hid_driver_active[i]) {
			if (hid_driver_active[i]) {
				DBGSerial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
				hid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				hid_driver_active[i] = true;

				const uint8_t* psz = hiddrivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = hiddrivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = hiddrivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);

				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}

#ifdef LATER
					if (joystick1.joystickType() == JoystickController::PS3_MOTION) {
						DBGSerial.println("  PS3 Motion detected");
						PS3_MOTION_timer = millis();  // set time for last event
						PS3_MOTION_tried_to_pair_state = 0;
					}
#endif
				}

			}
		}
	}
	
#if defined(BLUETOOTH)
	// Then Bluetooth devices
	for (uint8_t i = 0; i < CNT_BTHIDDEVICES; i++) {
		if (*bthiddrivers[i] != bthid_driver_active[i]) {
			if (bthid_driver_active[i]) {
				DBGSerial.printf("*** BTHID Device %s - disconnected ***\n", hid_driver_names[i]);
				bthid_driver_active[i] = false;
			}
			else {
				DBGSerial.printf("*** BTHID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
				bthid_driver_active[i] = true;

				const uint8_t* psz = bthiddrivers[i]->manufacturer();
				if (psz && *psz) DBGSerial.printf("  manufacturer: %s\n", psz);
				psz = bthiddrivers[i]->product();
				if (psz && *psz) DBGSerial.printf("  product: %s\n", psz);
				psz = bthiddrivers[i]->serialNumber();
				if (psz && *psz) DBGSerial.printf("  Serial: %s\n", psz);
				// See if this is our joystick object...
				if (hiddrivers[i] == &joystick1) {
					DBGSerial.printf("  Joystick type: %d\n", joystick1.joystickType());
					switch (joystick1.joystickType()) {
					case JoystickController::PS4:
						BTN_MASKS = PS4_BTNS;
						break;
					default:
					case JoystickController::PS3:
						BTN_MASKS = PS3_BTNS;
						break;
					}
				}
			}
		}
	}
#endif
}


//================================================================================
#ifdef OPT_TERMINAL_MONITOR_IC
// Optional stuff to allow me to have Input device debug support
//==============================================================================
// ShowTerminalCommandList: Allow the Terminal monitor to call the servo driver
//      to allow it to display any additional commands it may have.
//==============================================================================
void USBPSXController::ShowTerminalCommandList(void)
{
	DBGSerial.println(F("J - Show Joystick data"));
}

boolean USBPSXController::ProcessTerminalCommand(byte *psz, byte bLen)
{
	if ((bLen == 1) && ((*psz == 'j') || (*psz == 'J'))) {
		_fDebugJoystick = !_fDebugJoystick;
		if (_fDebugJoystick) DBGSerial.println("\n*** Joystick Debug ON ***");
		else DBGSerial.println("\n*** Joystick Debug OFF ***");
		return true;
	}
	return false;
}
#endif

//==============================================================================
//==============================================================================
//Send message back to remote
void USBPSXController::SendMsgs(byte Voltage, byte CMD, char Data[21]) {
#ifdef DBGSerial
	DBGSerial.printf("void USBPSXController::SendMsgs %u %u %s\n", Voltage, CMD, Data);
#endif
	// TODO, output to optional display
}
