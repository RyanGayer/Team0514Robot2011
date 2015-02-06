/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

import edu.mphs.first.interfaces.RobotInterface;
/**
 *
 * @author Corey
 */
public class PIDDriveUtil {
    PIDController moveController, turnController;
    double desiredAngle, desiredDistance;
    public PIDDriveUtil()
    {
        moveController = new PIDController(.6, 1.5, .1);
        turnController = new PIDController(.6, 1.5, .1);
        this.desiredAngle = 0;
    }

    public void setPath(boolean forkStraight, boolean leftRight)
    {
        if(forkStraight)
        {
            if(leftRight)
            {
                desiredAngle = RobotInterface.AUTO_ANGLE_L;
            }
            else
            {
                desiredAngle = RobotInterface.AUTO_ANGLE_R;
            }
            desiredDistance = 2400;
        }
        else
        {
            desiredDistance = 2400;
            desiredAngle = 0;
        }
    }

    public void setDesiredAngle(double desiredAngle)
    {
        this.desiredAngle = desiredAngle;
    }
    
    public boolean PIDDrive(Encoder xEncoder, Encoder yEncoder, Gyro robotGyro, RobotDrive roboDrive, Timer roboTimer)
    {
        if(Math.sqrt(xEncoder.get() * xEncoder.get() + yEncoder.get() * yEncoder.get()) < desiredDistance)
        {
            if(Math.sqrt(xEncoder.get() * xEncoder.get() + yEncoder.get() * yEncoder.get()) > desiredDistance * .1)
            {
                roboDrive.mecanumDrive_Polar(.5, moveController.returnPID(Math.tan(xEncoder.get()/yEncoder.get()), desiredAngle, roboTimer), turnController.returnPID(robotGyro.getAngle(), 0, roboTimer));
            }
            else
            {
                roboDrive.mecanumDrive_Polar(.5, desiredAngle, turnController.returnPID(robotGyro.getAngle(), 0, roboTimer));
            }
            return false;
        }
        else
        {
            roboDrive.mecanumDrive_Polar(0, 0, 0);
            return true;
        }
    }
}
