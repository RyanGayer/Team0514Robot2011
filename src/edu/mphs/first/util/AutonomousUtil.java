/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;


/**
 *
 * @author marnold
 */
public class AutonomousUtil {
    boolean onLine, followLine, onY, LeftRight, atPeg, pegDeployed;
    AnalogChannel pot;
    int m_pegHeight, m_autoStep;
    int[] PEG_POS_F, PEG_POS_S;

    public void AutonomousUtil(){
        m_pegHeight = 0;
        onLine = false;
        followLine = false;
        atPeg = false;
        pegDeployed = false;
    }
public void manageAutoBot(int m_autoStep, RobotDrive drive, String LeftorRight, String StraightorFork,
                    Encoder m_yEncoder, Encoder xEncoder,
                    DigitalInput leftSensor, DigitalInput centerSensor, DigitalInput rightSensor,
                    Gyro gyro,
                    LineTrackerUtil m_lineTracker, TrackerUtil m_trackUtil, GripUtil m_grip,
                    ForkliftUtil m_forkLift){

    switch (m_autoStep){
        case 0:
                m_lineTracker.manageTracker(drive, LeftorRight, StraightorFork,
                              leftSensor, centerSensor, rightSensor,
                              gyro, m_yEncoder, xEncoder);
/*
            // This code is Corey Holonomic Drive version
            if(StraightorFork.equalsIgnoreCase(RobotInterface.FORK)){
                onY = true;
            }else{
                onY = false;
            }
            if(LeftorRight.equalsIgnoreCase(RobotInterface.LEFT)){
                LeftRight = true;
            }else{
                LeftRight = false;
            }

            if(!onLine){
               onLine = m_trackUtil.moveToLine(centerSensor, drive, LeftRight);
            }
            if(onLine){
                followLine = m_trackUtil.followLine(centerSensor, drive, m_yEncoder, m_xEncoder, onY, LeftRight);
            }
 *
 */
        case 2:
        if(StraightorFork.equalsIgnoreCase(RobotInterface.STRAIGHT)){
            if(m_yEncoder.get() < RobotInterface.AUTO_ROLL_TO_WALL){
                m_lineTracker.manageTracker(drive, LeftorRight, StraightorFork,
                              leftSensor, centerSensor, rightSensor,
                              gyro, m_yEncoder, xEncoder);
            }
        }
        break;
        case 4:
            if(StraightorFork.equalsIgnoreCase(RobotInterface.STRAIGHT)){
                if(LeftorRight.equalsIgnoreCase(RobotInterface.LEFT)){
                    drive.arcadeDrive(RobotInterface.GET_AWAY_SPEED, RobotInterface.GET_AWAY_LEFT);
                }else{
                    drive.arcadeDrive(RobotInterface.GET_AWAY_SPEED, RobotInterface.GET_AWAY_RIGHT);
                }
            }else{
                if(LeftorRight.equalsIgnoreCase(RobotInterface.LEFT)){
                    drive.arcadeDrive(RobotInterface.GET_AWAY_SPEED, RobotInterface.GET_AWAY_RIGHT);
                }else{
                    drive.arcadeDrive(RobotInterface.GET_AWAY_SPEED, RobotInterface.GET_AWAY_LEFT);
                }
            }
        break;
    }
}

public boolean manageAutoLift(int m_autoStep, String StraightorFork,
                     AnalogChannel pot, Victor m_liftVictor, ForkliftUtil m_forkLift){
    switch (m_autoStep){
        case 0:
            m_pegHeight = RobotInterface.PEG_HIGH_BOTTOM;
            atPeg = m_forkLift.manageForklift(m_pegHeight, m_liftVictor, pot);
        break;
        case 2:
            if (StraightorFork.equalsIgnoreCase(RobotInterface.STRAIGHT)){
                m_pegHeight = RobotInterface.PEG_HIGH_S_LOW;
            }else{
                m_pegHeight = RobotInterface.PEG_HIGH_F_LOW;
            }
            atPeg = m_forkLift.manageForklift(m_pegHeight, m_liftVictor, pot);
        break;
    }
    return atPeg;
}

public boolean resetAutoLift(AnalogChannel pot, Victor m_liftVictor, ForkliftUtil m_forkLift){

    atPeg = m_forkLift.manageForklift(RobotInterface.PEG_HIGH_S_LOW, m_liftVictor, pot);

    return atPeg;
  }


public boolean manageAutoGrip(int m_autoStep, Solenoid m_wristCntrl, Solenoid m_gripCntrl,
                                DoubleSolenoid m_armCntrl, Victor m_liftVictor, GripUtil m_grip,
                                int loopCounter, int processSwitch){
           pegDeployed = m_grip.manageAutoGrip(m_autoStep, m_wristCntrl, m_gripCntrl, 
                                               m_armCntrl, m_liftVictor, loopCounter,
                                               processSwitch);
    return pegDeployed;
}
}
