/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Gyro;

/**
 *
 * @author Developer
 */
public class DriveUtil {
int driveMode;
double squaredInput;
String msg;

    public DriveUtil()
    {
        driveMode = 0;
        squaredInput = 0;
    }

    private void arcadeDrive(Joystick stickOne, RobotDrive roboDrive)
    {
        roboDrive.arcadeDrive(stickOne.getX(), stickOne.getY(), false);
    }

    private void tankDrive(Joystick stickOne, Joystick stickTwo, RobotDrive roboDrive)
    {
        roboDrive.tankDrive(-stickTwo.getY(), stickOne.getY());
    }

    private void holonomicDriveP(Joystick stickOne, Joystick stickTwo, RobotDrive roboDrive)
    {
        if(stickOne.getThrottle() > 0)
        {
            roboDrive.mecanumDrive_Polar(stickTwo.getMagnitude(), stickTwo.getDirectionDegrees(), stickOne.getX());
        }
        else
        {
            if(stickOne.getTwist() > 0)
            {
                squaredInput = stickOne.getTwist() * stickOne.getTwist();
            }
            else
            {
                squaredInput = -(stickOne.getTwist() * stickOne.getTwist());
            }
            roboDrive.mecanumDrive_Polar(stickOne.getMagnitude(), stickOne.getDirectionDegrees(), squaredInput);
        }
    }

    private void holonomicDriveC(Joystick stickOne, Joystick stickTwo, RobotDrive roboDrive, Gyro roboGyro)
    {
        if(stickOne.getThrottle() > 0)
        {
            roboDrive.mecanumDrive_Cartesian(stickOne.getX(), stickOne.getY(), stickTwo.getX(), roboGyro.getAngle());
        }
        else
        {
            roboDrive.mecanumDrive_Cartesian(stickOne.getX(), stickOne.getY(), stickOne.getTwist(), roboGyro.getAngle());
        }
    }

    private int getDriveMode(Joystick stickOne, int currentDrive, Gyro roboGyro)
    {
        if(stickOne.getRawButton(RobotInterface.TANK_MODE))
        {
            return 0;
        }
        if(stickOne.getRawButton(RobotInterface.ARCADE_MODE))
        {
            return 1;
        }
        if(stickOne.getRawButton(RobotInterface.POLAR_MODE))
        {
            return 2;
        }
        if(stickOne.getRawButton(RobotInterface.CARTESIAN_MODE) && !roboGyro.equals(null))
        {
            return 3;
        }
        return currentDrive;
    }

    public void manageDrive(RobotDrive robotDrive, Gyro robotGyro, Joystick joystickOne, Joystick joystickTwo)
    {
        driveMode = getDriveMode(joystickOne, driveMode, robotGyro);

        if(driveMode == 0)
        {
            tankDrive(joystickOne, joystickTwo, robotDrive);
        }
        else
        {
            if(driveMode == 1)
            {
                arcadeDrive(joystickOne, robotDrive);
            }
            else
            {
                if(driveMode == 2)
                {
                    holonomicDriveP(joystickOne, joystickTwo, robotDrive);
                }
                else
                {
                    holonomicDriveC(joystickOne, joystickTwo, robotDrive, robotGyro);
                }
            }
        }
        if(joystickOne.getRawButton(RobotInterface.GRYO_RESET) && !robotGyro.equals(null))
        {
            robotGyro.reset();
        }

    }
}