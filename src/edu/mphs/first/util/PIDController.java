// If you are here to fix the tuning code, turn back now.  Cut your losses.

// TIMES SOMEONE HAS TRIED TO FIX TUNING CODE:  0

package edu.mphs.first.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

public class PIDController {

double kP, kI, kD, accumulatedError, prevError, prevTime, pidValue, changeValue;
boolean enabled, changed;

    public PIDController(double kP, double kI, double kD)
    {
        prevError = 0;
        accumulatedError = 0;
        pidValue = 0;
        changeValue = .1;
        enabled = false;
        changed = false;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private double getP(double current, double setpoint)
    {
        return kP * (setpoint - current);
    }

    private double getI(double current, double setpoint)
    {
        return kI * (accumulatedError + setpoint - current);
    }

    private double getD(double current, double setpoint, Timer robotTimer)
    {
        return (setpoint - current - prevError) / (robotTimer.get() - prevTime);
    }

    public double returnPID(double current, double setpoint, Timer robotTimer)
    {
        if(enabled)
        {
            pidValue = getP(current, setpoint) + getI(current, setpoint) + getD(current, setpoint, robotTimer);
        }
        else
        {
            pidValue = 0;
        }
        prevTime = robotTimer.get();
        return pidValue;
    }

    public void enablePID()
    {
        enabled = true;
    }

    public void disablePID()
    {
        enabled = false;
    }

    public void pidTune(Joystick pidTuner)
    {
        if(pidTuner.getRawButton(5))
        {
            changeValue = changeValue * 1.1;
            System.out.println("Change Value: " + changeValue);
        }

        if(pidTuner.getRawButton(6))
        {
            if(changeValue > 0)
            {
                changeValue = changeValue * .9;
            }
            System.out.println("Change Value: " + changeValue);
        }

        if(pidTuner.getRawButton(7))
        {
            kP = kP + changeValue;
            changed = true;
        }

        if(pidTuner.getRawButton(8))
        {
            kP = kP - changeValue;
            changed = true;
        }

        if(pidTuner.getRawButton(9))
        {
            kI = kI + changeValue;
            changed = true;
        }

        if(pidTuner.getRawButton(10))
        {
            kI = kI - changeValue;
            changed = true;
        }

        if(pidTuner.getRawButton(11))
        {
            kD = kD + changeValue;
            changed = true;
        }

        if(pidTuner.getRawButton(12))
        {
            kD = kD - changeValue;
            changed = true;
        }

        if(changed)
        {
            System.out.println("kP = " + kP + " kI = " + kI + " kD = " + kD);
            changed = false;
        }
    }
    
    public void displayPIDSettings()
    {
        System.out.println("kP = " + kP + " kI = " + kI + " kD = " + kD);
    }
}