/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author Corey
 */
public class TrackerUtil {

    boolean[] sequence;
    boolean prevState;
    String msg;


    public TrackerUtil()
    {
        sequence = new boolean[4];
        prevState = false;
    }

    public boolean moveToLine(DigitalInput centerSensor, RobotDrive roboDrive, boolean leftRight)
    {
        if(leftRight)
        {
            if(centerSensor.get() == true && prevState == false)
            {
                roboDrive.mecanumDrive_Polar(0, 0, 0);
                return true;
            }
        }
        else
        {
            if(!centerSensor.get())
            {
                roboDrive.mecanumDrive_Polar(0, 0, 0);
                return true;
            }
        }
        roboDrive.mecanumDrive_Polar(RobotInterface.AUTONOMOUS_GETLINE_SPEED, 90, 0);
        return false;
    }

    public boolean followLine(DigitalInput centerSensor, RobotDrive roboDrive,
            Encoder yEncoder, Encoder xEncoder,
            boolean onY, boolean leftRight)
    {
        if(onY)
        {
            if(Math.sqrt(Double.parseDouble(Integer.toString(yEncoder.get()^2 + xEncoder.get()^2))) >
                    (RobotInterface.AUTO_Y1_TRAVEL_DISTANCE + RobotInterface.AUTO_Y2_TRAVEL_DISTANCE))
            {
                roboDrive.mecanumDrive_Polar(0, 0, 0);
                return true;
            }
        }
        else
        {
            if(Math.sqrt(Double.parseDouble(Integer.toString(yEncoder.get()^2 + xEncoder.get()^2))) >
                    RobotInterface.AUTO_LINE_TRAVEL_DISTANCE)
            {
                roboDrive.mecanumDrive_Polar(0, 0, 0);
                return true;
            }
        }
        if(centerSensor.get() == true)
        {
            if(leftRight)
            {
                roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 345, 0);
            }
            else
            {
                roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 15, 0);
            }
        }
        else
        {
            if(onY)
            {
                if(leftRight)
                {
                    roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 40, 0);
                }
                else
                {
                    roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 320, 0);
                }
            }
            else
            {
                if(leftRight)
                {
                    roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 15, 0);
                }
                else
                {
                    roboDrive.mecanumDrive_Polar(RobotInterface.TRACKER_SPEED, 345, 0);
                }
            }
        }
        return false;
    }
}
