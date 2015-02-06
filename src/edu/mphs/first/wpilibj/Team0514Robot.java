/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.mphs.first.wpilibj;


// Start MPHS Robotics Team 514 Class Imports
import edu.mphs.first.interfaces.RobotInterface;
import edu.mphs.first.util.DriveUtil;
import edu.mphs.first.util.DashboardUtil;
import edu.mphs.first.util.CameraUtil;
import edu.mphs.first.util.DeployUtil;
import edu.mphs.first.util.ForkliftUtil;
import edu.mphs.first.util.GripUtil;
import edu.mphs.first.util.AutonomousUtil;
import edu.mphs.first.util.LineTrackerUtil;
import edu.mphs.first.util.TrackerUtil;
import edu.mphs.first.util.PIDDriveUtil;
// End MPHS Robotics Team 514 Class Imports
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.camera.AxisCamera;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.RobotDrive;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team0514Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
     /** Team 0514 Notes:
      * 
      * Global Variables:
      * All Global Variables MUST be placed in the RobotInterface.java.  If you
      * are defining a STATIC FINAL variable then it is a candidate for being
      * placed in the Interface!
      *
      * Common Objects:
      * All Object declarations MUST be placed in the Team0514Robot.java as
      * is being done below.
      *
      */
     // Declare variable for the robot drive system
	DriveUtil m_robotDrive;
        RobotDrive drive;

     // Declare ForkLift variables
        ForkliftUtil m_robotLift;
        AnalogChannel m_pot;
        int m_potValue, m_autoStep;
        //Declare DashboardUtil
        DashboardUtil trackerDashboard;

	// Declare variables for the two joysticks being used
	Joystick m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick m_leftStick;			// joystick 2 (tank left stick)
	Joystick m_controller;			// joystick 3 (tank left stick)

        // Declare a the Compressor object
	Compressor m_comp;                     

        // Declare a variable to use to access the Encoder object
        boolean reverseDirection = false;
        private EncodingType m_encodingType = EncodingType.k4X;
        Encoder m_xEncoder, m_yEncoder;                     // Encoder object
        int m_xyValue;


        // Declare Timer object and variables
        private double m_accumulatedTime;
        boolean m_running;
	Timer m_timer;                     // Timer object


	// Local variables to count the number of periodic loops performed
	int m_autoPeriodicLoops;
        int m_myAutoPeriodicLoops;
        int m_myPeriodicLoops;
	int m_disabledPeriodicLoops;
	int m_telePeriodicLoops;
        boolean m_autonomous, atPeg, pegDeployed, atWall, atBottom, onTarget, 
                armReset, pegGrab, stepOne;
        String msg;
        String LeftorRight, StraightorFork;
        int[] PEG_POS_F;
        int[] PEG_POS_S;

        // Instantiate the Jaguars on the left and right side and Victor for Winch
        Jaguar m_lfJaguar, m_lrJaguar;
        Jaguar m_rfJaguar, m_rrJaguar;
        Victor m_liftVictor;

        //Instantiate MicroBot Deploy Object
        DeployUtil m_microBot;
        // Mr. Gerkey wants this off the button press on the game controller
        //DigitalInput m_standSwitch;

        //Instantiate Gripper Object
        GripUtil m_grip;

        //Instantiate AutonomousUtil Object
        AutonomousUtil m_autoBot;
        DigitalInput m_leftRight, m_straightFork;

        //Line Tracker Sensors
        DigitalInput m_leftSensor, m_centerSensor, m_rightSensor;

        //Add Camera Objects
        //AxisCamera cam;

        Gyro gyro;

        //Remove the turnController for the competion.
        //PIDController turnController;

        //Servo servoX, servoY;
        //CameraUtil camera;
        double pidOutput, angle;

        //Define Solenoids
        DoubleSolenoid m_armCntrl;//
        DoubleSolenoid m_trayCntrl;//
        //DoubleSolenoid m_standCntrl;//
        Solenoid m_wristCntrl, m_gripCntrl;//

        LineTrackerUtil m_lineTracker;
        TrackerUtil m_tracker;

        PIDDriveUtil m_PIDDriver;


    public Team0514Robot() {
     /**
     * Constructor for this "Team0514Robot" Class.
     *
     * The constructor creates all of the objects used for the different inputs and outputs of
     * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
     * providing named objects for each of the robot interfaces.
     */

        
        // Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
	m_rightStick = new Joystick(RobotInterface.RIGHT_Z_JOYSTICK);
	m_leftStick = new Joystick(RobotInterface.LEFT_JOYSTICK);
        m_controller = new Joystick(RobotInterface.CONTROLLER);
        m_lfJaguar = new Jaguar(RobotInterface.LF_JAGUAR);
        m_lrJaguar = new Jaguar(RobotInterface.LR_JAGUAR);
        m_rfJaguar = new Jaguar(RobotInterface.RF_JAGUAR);
        m_rrJaguar = new Jaguar(RobotInterface.RR_JAGUAR);


        //Create the Compressor
        m_comp = new Compressor (RobotInterface.COMP_P_SWITCH_CHANNEL,
                                 RobotInterface.COMP_C_RELAY_CHANNEL);

        //Instantiate the Encoder
        m_xEncoder = new Encoder (RobotInterface.ENCODER_X_SLOT_A,
                                RobotInterface.ENCODER_X_CHANNEL_A,
                                RobotInterface.ENCODER_X_SLOT_B,
                                RobotInterface.ENCODER_X_CHANNEL_B,
                                reverseDirection,m_encodingType);

        m_yEncoder = new Encoder (RobotInterface.ENCODER_Y_SLOT_A,
                                RobotInterface.ENCODER_Y_CHANNEL_A,
                                RobotInterface.ENCODER_Y_SLOT_B,
                                RobotInterface.ENCODER_Y_CHANNEL_B,
                                reverseDirection,m_encodingType);

        // Instantiate all ForkLiftUtil Objects
        m_pot = new AnalogChannel(RobotInterface.POT1_SLOT);
        m_pot.resetAccumulator();
        m_liftVictor = new Victor(RobotInterface.FORK_LIFT);
        m_robotLift = new ForkliftUtil(m_pot);
        m_armCntrl = new DoubleSolenoid(RobotInterface.PNEU_SLOT,
                RobotInterface.ARM_FORWARD, RobotInterface.ARM_BACK);
        PEG_POS_F = new int[5];
        PEG_POS_S = new int[5];

        //Instantiate Line Tracker Sensors
        m_leftSensor = new DigitalInput(RobotInterface.photoSensorSwitch, RobotInterface.LEFT_TRACK_SENSOR);
        m_centerSensor = new DigitalInput(RobotInterface.photoSensorSwitch, RobotInterface.MIDDLE_TRACK_SENSOR);
        m_rightSensor = new DigitalInput(RobotInterface.photoSensorSwitch, RobotInterface.RIGHT_TRACK_SENSOR);


        // Initialize counters to record the number of loops completed in autonomous and teleop modes
	m_autoPeriodicLoops = 0;
        m_myAutoPeriodicLoops = 0;
        m_myPeriodicLoops = 0;
	m_disabledPeriodicLoops = 0;
	m_telePeriodicLoops = 0;

        // Instiantiate the Gyro
        gyro = new Gyro(RobotInterface.GYRO_SLOT, RobotInterface.GYRO_CHANNEL);

        // Initialize the relays

        //Instantiate Autonomous Mode Switches

        // Create a robot using standard right/left robot drive on PWMS 1 and 2 Only
        m_robotDrive = new DriveUtil();
        drive = new RobotDrive(m_lfJaguar, m_lrJaguar, m_rfJaguar, m_rrJaguar);

        // Instantiate the Dashboard
        trackerDashboard = new DashboardUtil();

        //Instantiate DeployUtil
        m_microBot = new DeployUtil();
        m_trayCntrl = new DoubleSolenoid(RobotInterface.PNEU_SLOT,
                RobotInterface.TRAY_OUT, RobotInterface.TRAY_BACK);
        
//        m_standCntrl = new DoubleSolenoid(RobotInterface.PNEU_SLOT,
//                RobotInterface.STAND_OUT, RobotInterface.STAND_BACK);
        //m_standSwitch = new DigitalInput(RobotInterface.StandSwitch);

        //Instantiate GripUtil;
        m_grip = new GripUtil();
        m_wristCntrl = new Solenoid(RobotInterface.PNEU_SLOT, RobotInterface.WRIST_PNEU);
        m_gripCntrl = new Solenoid(RobotInterface.PNEU_SLOT, RobotInterface.GRIP_PNEU);

        //Instantiate AutonomousUtil
        m_autoBot = new AutonomousUtil();
        m_leftRight = new DigitalInput(RobotInterface.LEFT_RIGHT);
        m_straightFork = new DigitalInput(RobotInterface.STRAIGHT_FORK);
        stepOne = false;


        //Instantiate Camera Objects
        /*
         *
        cam = AxisCamera.getInstance();
        camera = new CameraUtil();
        servoX = new Servo(RobotInterface.SERVO_X_AXIS);
        servoY = new Servo(RobotInterface.SERVO_Y_AXIS);
         */

        m_PIDDriver = new PIDDriveUtil();

        //Constructor...  Instantiate local objects
/*
 * Remove the turnController.  We will not use this PIDControl in the competion!
        turnController = new PIDController(.08, 0.0, 0.5, gyro, new PIDOutput() {

           public void pidWrite(double output) {

            drive.arcadeDrive(0, output);
        }
       }, .005);
        turnController.setInputRange(-360.0, 360.0);
        turnController.setTolerance(1 / 90. * 100);
        turnController.disable();
*/
        m_lineTracker = new LineTrackerUtil();
        m_tracker = new TrackerUtil();

        
        System.out.println("Team0514 Robot::Constructor Completed\n");
	}

	/********************************** Init Routines *************************************/

    public void robotInit() {
       Watchdog.getInstance().feed();
//       Watchdog.getInstance().setExpiration(1);
        //Establish Start Time
        m_timer = new Timer();
       //Reset for Run Timer
        m_timer.reset();
        m_running = false;

        LeftorRight = "L";
        StraightorFork = "S";
        m_xyValue = 0;

        //Print Log

        //Start the compressor thread
        m_comp.start();

        //Reset the gyro
        gyro.reset();

        //put the lift victor NOT in safety mode
        //m_liftVictor.setSafetyEnabled(false);
        
        //Load PEG Position Array
        for (int i=1;i <=4 ; i++){
                switch (i){
                    case 1:
                        PEG_POS_S[i] = RobotInterface.PEG_HIGH_BOTTOM;
                    break;
                    case 2:
                        PEG_POS_S[i] = RobotInterface.PEG_HIGH_S_LOW;
                    break;
                    case 3:
                        PEG_POS_S[i] = RobotInterface.PEG_HIGH_S_MID;
                    break;
                    case 4:
                        PEG_POS_S[i] = RobotInterface.PEG_HIGH_S_TOP;
                    break;
                }
        }
        for (int i=1;i <=4 ; i++){
                switch (i){
                    case 1:
                        PEG_POS_F[i] = RobotInterface.PEG_HIGH_BOTTOM;
                    break;
                    case 2:
                        PEG_POS_F[i] = RobotInterface.PEG_HIGH_F_LOW;
                    break;
                    case 3:
                        PEG_POS_F[i] = RobotInterface.PEG_HIGH_F_MID;
                    break;
                    case 4:
                        PEG_POS_F[i] = RobotInterface.PEG_HIGH_F_TOP;
                    break;
                }
        }
    }

  /* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   *  If we choose to replace the MAIN LOOP code we would override the 
   *  IterativeRobot.startCompetition() method.
   *  Right now we are NOT going to change that code!!
   * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   *  public void startCompetition() {
   *      super.startCompetition();
   *  }
 */

    public void autonomousInit() {
        m_autoStep = 0;
        atPeg = false;
        armReset = false;
        pegGrab = false;
        pegDeployed = false;
        atWall = false;
        atBottom = false;
        onTarget = false;

        //Start the Encoder
            m_xEncoder.start();
            m_yEncoder.start();
            m_xEncoder.reset();
            m_yEncoder.reset();

        //Enable the turnController
//            turnController.enable();
            pidOutput = 0;
        
        m_autonomous = true;
        m_autoPeriodicLoops = 0;
        m_myAutoPeriodicLoops = 0;
        m_myPeriodicLoops = 0;

        //Start the Autonomous Timer
            m_timer.stop();
            m_timer.reset();
            m_timer.start();
            m_accumulatedTime = 0;

            gyro.reset();

        // Read the new switch to determine what area of the arena we are operating in.
        // Set the default to 1 and read switches to determine override if any.
        if (m_leftRight.get() == true){
            LeftorRight = RobotInterface.LEFT;
            angle = RobotInterface.AUTO_ANGLE_L;
        }else{
            LeftorRight = RobotInterface.RIGHT;
            angle = RobotInterface.AUTO_ANGLE_R;
        }
        if (m_straightFork.get() == true){
            StraightorFork = RobotInterface.STRAIGHT;
        }else{
            StraightorFork = RobotInterface.FORK;
        }
//            camera.cameraInit(cam, gyro, m_controller, servoX, servoY, m_autonomous);

        //Set the starting position of the Grip, Wrist, Arm, Tray and Stand Solenoids
            m_gripCntrl.set(true);
            m_wristCntrl.set(false);
            m_armCntrl.set(DoubleSolenoid.Value.kReverse);
//            m_standCntrl.set(DoubleSolenoid.Value.kReverse);
            m_trayCntrl.set(DoubleSolenoid.Value.kReverse);
//            m_gripCntrl.set(false);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        // Get Instance of Watchdog and feed
        Watchdog.getInstance().feed();
         //Increment Counter
        m_autoPeriodicLoops++;


        //Place our autonomousPeriodic() code here!
        m_accumulatedTime = m_timer.get();
            if (m_accumulatedTime >= RobotInterface.M_AUTO_TIMELIMIT) {
               //Stop the Robot!
                m_autonomous = false;
                msg = "Auto:OFF";
            }else{
                msg = "Auto:ON ";
                }
            if (m_autonomous){
                msg = "AutoRunTime:" + m_accumulatedTime;
                AutonomousRobot();
            }
 
    }

    public void teleopInit() {
        m_autonomous = false;
        atPeg = false;
        pegDeployed = false;
        atWall = false;
        atBottom = false;
        onTarget = false;

        m_telePeriodicLoops = 0;
        //Start the Encoder
            m_xEncoder.start();
            m_yEncoder.start();
            m_xEncoder.reset();
            m_yEncoder.reset();

        //Enable the turnController
//            turnController.enable();
            pidOutput = 0;

        //Reset the Timer for TeleOp Use
            m_timer.stop();
            m_timer.reset();
            m_timer.start();
            m_accumulatedTime = 0;
            m_myPeriodicLoops = 0;
            gyro.reset();
            
        //Initialize the Camera for Teleop
  //          camera.cameraInit(cam, gyro, m_controller, servoX, servoY, m_autonomous);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        // Get Instance of Watchdog and feed
        Watchdog.getInstance().feed();
        // Get Match Running Time
        m_accumulatedTime = m_timer.get();
        //Increment Counter
        m_telePeriodicLoops++;
        //Place our teleopPeriodic() code here!
 
        m_robotDrive.manageDrive(drive, gyro, m_rightStick, m_leftStick);

        //Add Controls and Calls for the CameraUtil
        /*
        *
         pidOutput = camera.manageCamera(m_autonomous, cam, gyro, drive, m_controller,
                                        servoX, servoY, trackerDashboard);
        */

        //Add Controls and Calls for the ForkLiftUtil
        //Controls are all m_controller buttons
        m_robotLift.manageForklift(m_controller, m_rightStick, m_liftVictor, m_pot,
                                   PEG_POS_F, PEG_POS_S);

        //Add Controls and Calls for the GripUtil
        //Controls are all m_controller buttons
            m_grip.manageGrip(m_wristCntrl, m_gripCntrl, m_armCntrl, m_controller);            
        
        //Add Controls and Calls for the CameraUtil
        /*Remove the turnController from the competion!
         *
        if(m_controller.getRawButton(RobotInterface.TRUE_UP) && (!onTarget)){
            if(turnController.isEnable()){
                PIDCntrlCam();
            }else{
                turnController.enable();
            }
        }
         */
        //Add Controls and Calls for the DeployUtil
        //Controls are a combination of the m_timer and m_controller buttons
        if(m_accumulatedTime == RobotInterface.DEPLOY_TIME){
                m_microBot.manageDeploy(m_controller, m_trayCntrl);
        }
        //Debug Statements to help calibrate on Thursday...
        /*
         * 
        System.out.println("Gryo getAngle = " + gyro.getAngle() +
                           " X-Encoder = " + m_xEncoder.get() + 
                           " Y-Encoder = " + m_yEncoder.get() +
                           " Potentiometer = " + m_pot.getValue());
         */
   }

    public void disabledInit() {

        //Place our disabledInit() code here!
        m_disabledPeriodicLoops = 0;
        m_yEncoder.stop();
        m_yEncoder.reset();
        m_xEncoder.stop();
        m_xEncoder.reset();
        m_timer.stop();
        m_timer.reset();
/*Remove the turnController from the competion...
 * 
        turnController.reset();
        turnController.disable();
 */
    }

    public void disabledPeriodic() {
        // Get Instance of Watchdog and feed
        Watchdog.getInstance().feed();
        //Increment Counter
        m_disabledPeriodicLoops++;
        //Place our disabledPeriodic() code here!
    }
    private void AutonomousRobot(){
        switch(m_autoStep){
            case 0:
                //Position Arm Forward, Low and Open
                AutoPositionArm(RobotInterface.PICK_UP);
            break;
            case 1:
                //Grab and Pick Up Game Piece
                AutoDeployPiece(RobotInterface.PICK_UP);
            break;
            case 2:
                //Hang Up Game Piece on Peg
                AutoPositionArm(RobotInterface.HANG_UP);
            break;
            case 3:
                //Hang Up Game Piece on Peg
                AutoDeployPiece(RobotInterface.HANG_UP);
            break;
            case 4:
                //Reset the Arm and Grip
                AutoPositionArm(RobotInterface.HANG_UP);
            break;
        }
    }
    private void AutoPositionArm(int processSwitch){
        if(processSwitch == RobotInterface.PICK_UP){
            m_armCntrl.set(DoubleSolenoid.Value.kForward);
            atPeg = m_autoBot.manageAutoLift(m_autoStep, StraightorFork, m_pot, m_liftVictor,
                                             m_robotLift);
            if(atPeg){
                m_autoStep = 1;
            }
        }
        if((processSwitch == RobotInterface.HANG_UP) && (m_autoStep == 2)){
            armReset = m_autoBot.manageAutoLift(m_autoStep, StraightorFork, m_pot, m_liftVictor,
                                             m_robotLift);
            if(armReset){
                m_autoStep = 3;
            }            
        }
        if((processSwitch == RobotInterface.HANG_UP) && (m_autoStep == 4)){
            m_wristCntrl.set(false);
            m_gripCntrl.set(true);
            armReset = m_autoBot.resetAutoLift(m_pot, m_liftVictor, m_robotLift);
            m_armCntrl.set(DoubleSolenoid.Value.kReverse);
            if(armReset){
                m_autoStep = 5;
            }            
        }
    }
    private void AutoDeployPiece(int processSwitch){
        //m_autoStep = 1
        if(processSwitch == RobotInterface.PICK_UP){
            m_myAutoPeriodicLoops++;
            pegGrab = m_autoBot.manageAutoGrip(m_autoStep, m_wristCntrl, m_gripCntrl, m_armCntrl,
                                               m_liftVictor, m_grip, m_myAutoPeriodicLoops,
                                               processSwitch);
               if(pegGrab){
                  m_autoStep = 2;
                  m_myAutoPeriodicLoops = 0;
            }
        }
        //m_autoStep = 2
        if(processSwitch == RobotInterface.HANG_UP){
            m_myAutoPeriodicLoops++;
            pegDeployed = m_autoBot.manageAutoGrip(m_autoStep, m_wristCntrl, m_gripCntrl, m_armCntrl,
                                               m_liftVictor, m_grip, m_myAutoPeriodicLoops,
                                               processSwitch);
               if(pegDeployed){
                  m_autoStep = 4;
                  m_myAutoPeriodicLoops = 0;
            }
        }
    }
}