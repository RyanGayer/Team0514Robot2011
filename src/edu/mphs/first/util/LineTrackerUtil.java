/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2010. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.mphs.first.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Encoder;


public class LineTrackerUtil {
    //All variables with Class Scope (seen by all methods in this class) are
    //defined here...
  // Declare a single binary value for the three tracking sensors.
        int binaryValue;
        int priorBinaryValue;
        // Adjust the turn of the robot drives to stay on line.
        double steeringGain, rotate;
        boolean atCross = false; // if robot has arrived at end
        double speed, turn;
        //Declare the RobotDrive Object
    //Declare general purpose variables (boolean, int, double, String, etc...)
    boolean readLO;
    int leftVal;
    int middleVal;
    int rightVal;
    String msg;
    double p_direction;


    //Declare the default value for the steering gain (to turn the robot)
    double defaultSteeringGain = 0.5;
     
        

    public void trackerInit(){
            readLO = false;
    }
    public void manageTracker(RobotDrive drive, String m_direction, String m_straight,
                              DigitalInput leftSensor, DigitalInput centerSensor, DigitalInput rightSensor,
                              Gyro gyro, Encoder yEncoder, Encoder xEncoder) {
        
        tracker(drive, m_direction, m_straight, leftSensor, centerSensor, rightSensor, gyro, yEncoder, xEncoder);

    }



   private void tracker(RobotDrive drive, String m_direction, String m_straight,
                        DigitalInput leftSensor, DigitalInput centerSensor, DigitalInput rightSensor,
                        Gyro gyro, Encoder yEncoder, Encoder xEncoder){
       if (readLO){
                leftVal =  leftSensor.get()? 1 : 0;
                middleVal = centerSensor.get() ? 1 : 0;
                rightVal = rightSensor.get() ? 1 : 0;
            }else{
                leftVal = leftSensor.get() ? 0 : 1;
                middleVal = centerSensor.get() ? 0 : 1;
                rightVal = rightSensor.get() ? 0 : 1;
            }
        //Main Control
            if (m_direction.equalsIgnoreCase(RobotInterface.LEFT)) {
                binaryValue = leftVal * 4 + middleVal * 2 + rightVal;
                steeringGain = -defaultSteeringGain;
            } else {
                binaryValue = rightVal * 4 + middleVal * 2 + leftVal;
                steeringGain = defaultSteeringGain;
            }
            // get the default speed and turn rate at this time
            speed = .6;
            turn = steeringGain;
            // different cases for different line tracking sensor readings
            switch (binaryValue) {

                case 1:  // on line edge
                    turn = (-defaultSteeringGain * .75);
                    break;
                case 4:
                    turn = (defaultSteeringGain * .75);
                    break;
                case 2:
                    turn = 0;
                    break;
                case 7:  // all sensors on (maybe at cross)
                    if(m_straight.equalsIgnoreCase(RobotInterface.FORK)){
                        if(yEncoder.get() > (RobotInterface.AUTO_Y1_TRAVEL_DISTANCE +
                                             RobotInterface.AUTO_Y2_TRAVEL_DISTANCE)){
                            atCross = true;
                            speed = 0;
                            turn = 0;
                        }
                    }else{
                        if(yEncoder.get() > RobotInterface.AUTO_LINE_TRAVEL_DISTANCE){
                            atCross = true;
                            speed = 0;
                            turn = 0;
                        }
                    }
                    break;
                case 0:  // all sensors off
                    //We lost the line....
                    if((priorBinaryValue == 1) || (priorBinaryValue == 3)){
                        turn = -defaultSteeringGain;
                    }
                    if((priorBinaryValue == 4) || (priorBinaryValue == 6)){
                        turn = defaultSteeringGain;                        
                    }
                break;
                case 3:
                    turn = -defaultSteeringGain;
                    break;
                case 6:
                    turn = defaultSteeringGain;
                    break;
                case 5:  // all other cases
                    turn = 0;
                break;
            }
            //Determine the Robot's Rotation by reading the Gyro
            //Set the appropriate compensating Rotation Value.
            // print current status for debugging
           // set the robot speed and direction

        if(RobotInterface.AUTO_DRIVE_MODE == RobotInterface.ARCADE_DRIVE){
            drive.arcadeDrive(speed, turn, false);
        }
        if(RobotInterface.AUTO_DRIVE_MODE == RobotInterface.POLAR_DRIVE){
            if(priorBinaryValue == 4 || priorBinaryValue == 6){
                p_direction = 350;
            }
            if(priorBinaryValue == 1 || priorBinaryValue == 3){
                p_direction = 80;
            }
            if(turn == 0){
                p_direction = 0;
            }
            if((speed == 0) && (turn == 0)){
                rotate = 0;
            }else{
            determineRotation(xEncoder);
            }
            drive.mecanumDrive_Polar(speed, p_direction, rotate);
        }
        if(RobotInterface.AUTO_DRIVE_MODE == RobotInterface.CARTESIAN_DRIVE){
            if((speed == 0) && (turn == 0)){
                rotate = 0;
            }else{
            determineRotation(xEncoder);
            drive.mecanumDrive_Cartesian(speed, turn, rotate, gyro.getAngle());
            }
        }

            if(binaryValue != 0){
                priorBinaryValue = binaryValue;
            }
   }
   private void determineRotation(Encoder xEncoder){

        double sway = xEncoder.get();
        //Use a Switch/Case in case you need different Rotation calculations for different drive modes...
        switch(RobotInterface.AUTO_DRIVE_MODE){
            default:
                if(sway > 10){
                    rotate = -((sway/10)*.5);
                }
                if(sway < -10){
                    rotate = ((sway/10)*.5);
                }
                if(rotate > .5){
                    rotate = .5;
                }
                if(rotate < -.5){
                    rotate = -.5;
                }
            break;
        }
   }
}
