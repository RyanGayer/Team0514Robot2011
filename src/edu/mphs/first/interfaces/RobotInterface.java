/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.interfaces;

/**
 * You are to use this interface to define all public static final variables
 * that the Robot will use.  The primary use of this interface is the define
 * all variables that represent device wiring configuration back to the cRIO
 * and Digital Side Car.  These are perfect candidates to take advantage of
 * the Interface pattern.  Use this for other variable declarations as well but
 * if you are unsure, define it to your local class.method() and we can discuss
 * it later.
 *
 * @author marnold
 */
public interface RobotInterface {
    
//------------------------Misc Runtime Config Parameters-------------------------------
    //Constants
        //0 = Off, 1 = On.  In the future there may be other levels...
        public static final int M_DEBUG_LEVEL = 0;
        public static final int M_LCD_LEVEL = 0;

        // Declare Drive Mode selection
	public static final int UNINITIALIZED_DRIVE = 0;
        public static final int AUTO_DRIVE_MODE = 2;
	public static final int ARCADE_DRIVE = 1;
	public static final int TANK_DRIVE = 0;
        public static final int POLAR_DRIVE = 2;
        public static final int CARTESIAN_DRIVE = 3;

        // Declare Joystick variables
        public static final int NUM_JOYSTICK_BUTTONS = 16;
        public static final int LEFT_JOYSTICK = 2;
        public static final int RIGHT_Z_JOYSTICK = 1;
        public static final int CONTROLLER = 3;

        // Declare Potentiometer Range Values

        // Joystick Button Mappings:
        //Right Stick (1)
        public static final int GRYO_RESET = 1;
        public static final int TANK_MODE = 3;
        public static final int ARCADE_MODE = 4;
        public static final int POLAR_MODE = 5;
        public static final int CARTESIAN_MODE = 6;
        //Controller (2)
        //Axis
        public static final int dPAD_PAN_X = 1;
        public static final int dPAD_PAN_Y = 5;
        //Buttons (10)
        public static final int ARM_OUT = 3;
        public static final int Pole_Selector = 4;
        public static final int SCROLL_HEIGHT = 2;
        public static final int SCROLL_UP = 5;    /* This is a combo - 2+6 = Up */
        public static final int SCROLL_DOWN = 7;  /* This is a combo - 2+8 = Down */
        //public static final int SPRING_BOT = 3;
        public static final int ARM_RETURN = 1;
        public static final int WRIST = 6;
        public static final int FORK_UP = 5;
        public static final int GRIP = 8;
        public static final int FORK_DOWN = 7;
        public static final int DEPLOY_BOT = 10;
        public static final int RETURN_BOT = 9;

        // Declare variables for each of the eight solenoid outputs
	public static final int NUM_SOLENOIDS = 8;

        // Autonomous Timer Constant Units Seconds
        public static final double M_AUTO_TIMELIMIT = 15;
        public static final double DEPLOY_TIME = 120;
        public static final String LEFT = "L";
        public static final String RIGHT = "R";
        public static final String STRAIGHT = "S";
        public static final String FORK = "F";

        // Autonomous Controls...

        //Calibrated distances to travel in Autonomous mode
             // 17.1 ticks = 1 inch.  205 ticks = 1 foot     //8890
        public static final int AUTO_Y1_TRAVEL_DISTANCE = 3587;
        public static final int AUTO_Y2_TRAVEL_DISTANCE = 1339;
        public static final int AUTO_LINE_TRAVEL_DISTANCE = 3472;
        public static final int AUTO_ROLL_TO_WALL = 410;
        public static final double AUTO_ANGLE_L = 74;
        public static final double AUTO_ANGLE_R = -74;

        // PID drive vs. Line Tracker- 0-1

        public static final boolean AUTO_TOWALL = true;
        
        public static final int PICK_UP = 0;
        public static final int HANG_UP = 1;

        public static final int PEG_HIGH_BOTTOM = 625;
        public static final int Max_Height = 110;
        public static final int Min_Height = 630;
        public static final int peg_1_inch = 6;
        // Straight Line - Pegs Higher values Lower
        public static final int PEG_HIGH_S_LOW = 542;
        public static final int PEG_HIGH_S_MID = 457;
        public static final int PEG_HIGH_S_TOP = 329;
        // Fork line - Pegs Lower values Higher
        public static final int PEG_HIGH_F_LOW = 542;
        public static final int PEG_HIGH_F_MID = 457;
        public static final int PEG_HIGH_F_TOP = 329;
        
        //Rates of Speed to use to start various Autonomous modes
        public static final double TRACKER_SPEED = .5;
        public static final double AUTONOMOUS_GETLINE_SPEED = .25;
        public static final double GET_AWAY_SPEED = .8;
        public static final double GET_AWAY_LEFT = -.65;
        public static final double GET_AWAY_RIGHT = .65;

        //CPU cycles limits and counts

//--------------------------------ROBOT PHYSICAL DEVICE MAPPING AREA-----------------------------------
    //Analog Bumper (Slot 1)
    // Declare variables for Potentiometer
        public static final int POT1_CHANNEL = 1;
        public static final int POT1_SLOT = 1;
        public static final int GYRO_CHANNEL = 2;
        public static final int GYRO_SLOT = 1;


    //Digital Side Car (Slot 4)
        //PWM OUT
        // Declare variables for the Jaguar Drives
        public static final int LF_JAGUAR = 1;
        public static final int LR_JAGUAR = 2;
        public static final int RF_JAGUAR = 3;
        public static final int RR_JAGUAR = 4;
        public static final int FORK_LIFT = 5;


        //Digital IO
        public static final int photoSensorSwitch = 4;
        public static final int StandSwitch = 11;
        // Declare a variable to use to access the Encoder object
        public static final int LEFT_TRACK_SENSOR = 1;
        public static final int MIDDLE_TRACK_SENSOR = 2;
        public static final int RIGHT_TRACK_SENSOR = 3;
        public static final int ENCODER_X_SLOT_A = 4;
        public static final int ENCODER_X_CHANNEL_A = 6;
        public static final int ENCODER_X_SLOT_B = 4;
        public static final int ENCODER_X_CHANNEL_B = 7;
        public static final int ENCODER_Y_SLOT_A = 4;
        public static final int ENCODER_Y_CHANNEL_A = 4;
        public static final int ENCODER_Y_SLOT_B = 4;
        public static final int ENCODER_Y_CHANNEL_B = 5;
        // Declare a variable to use to access the Compressor object
        public static final int COMP_P_SWITCH_SLOT = 4;
        public static final int COMP_P_SWITCH_CHANNEL = 8;
        public static final int SERVO_X_AXIS = 7;
        public static final int SERVO_Y_AXIS = 8;

        //Switches
        public static final int LEFT_RIGHT = 9;
        public static final int STRAIGHT_FORK = 10;

        //Relays
        public static final int COMP_C_RELAY_SLOT = 4;
        public static final int COMP_C_RELAY_CHANNEL = 8;


    //Pneumatic Bumper (Slot 8)
        public static final int PNEU_SLOT = 8;
        public static final int ARM_FORWARD = 1;
        public static final int ARM_BACK = 2;
        public static final int WRIST_PNEU = 3;
        public static final int GRIP_PNEU = 4;
        public static final int TRAY_OUT = 5;
        public static final int TRAY_BACK = 7;
        public static final int STAND_OUT = 6;
        public static final int STAND_BACK = 8;
        

}
