/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.mphs.first.interfaces.RobotInterface;


/**
 *
 * @author Developer
 */
public class ForkliftUtil {
int desiredValue;
int[] potentiometerPositions;
boolean previousUp, previousDown, atPeg, lowPegs;
String msg;


    public ForkliftUtil(AnalogChannel Potentiometer)
    {
        desiredValue = 1;
        previousUp = previousDown = false;
        atPeg = false;
        potentiometerPositions = new int[5];
    }

    private int getDesiredPotValue(Joystick controlStick, int currentDesiredValue, boolean prevUp, boolean prevDown)
    {
        if(controlStick.getRawButton(RobotInterface.FORK_UP) && prevUp == false)
        {
            if(currentDesiredValue != 4)
            {
                return currentDesiredValue + 1;
            }
            else
            {
                return currentDesiredValue;
            }
        }
        else
        {
            if(controlStick.getRawButton(RobotInterface.FORK_DOWN) && prevDown == false)
            {
                if(currentDesiredValue != 1)
                {
                    return currentDesiredValue - 1;
                }
                else
                {
                    return currentDesiredValue;
                }

            }
            else
            {
                return currentDesiredValue;
            }
        }
    }

    private void driveForklift(int currentDesiredValue, Joystick m_rightstick, int[] PEG_POS_S, int[] PEG_POS_F,
                                Victor m_liftVictor, AnalogChannel liftPot)
    {
        loadSelectedArray(m_rightstick, PEG_POS_S, PEG_POS_F);
        if(potentiometerPositions[currentDesiredValue] < liftPot.getValue() - 10)
        {
            m_liftVictor.set(1);
        }
        else
        {
            if(potentiometerPositions[currentDesiredValue] > liftPot.getValue() + 10)
            {
                m_liftVictor.set(-1);
            }
            else
            {
                m_liftVictor.set(0);
            }
        }
    }

    private void driveForklift(Joystick controlStick, Joystick rightStick, int[] PEG_POS_S, int[] PEG_POS_F,
                                Victor m_liftVictor, AnalogChannel liftPot)
    {
        loadSelectedArray(controlStick, PEG_POS_S, PEG_POS_F);
        if(controlStick.getRawButton(RobotInterface.SCROLL_HEIGHT)){
            if(controlStick.getRawButton(RobotInterface.SCROLL_UP) &&
                    RobotInterface.Max_Height <= liftPot.getValue()){
                m_liftVictor.set(1);
        }else{
            if(controlStick.getRawButton(RobotInterface.SCROLL_DOWN) &&
                    RobotInterface.Min_Height >= liftPot.getValue()){
                m_liftVictor.set(-1);
            }else{
                m_liftVictor.set(0);
            }
        }
        }
    }

    public void manageForklift(Joystick controlStick, Joystick m_rightstick, Victor m_liftVictor,
                                AnalogChannel liftPot, int[] PEG_POS_F, int[] PEG_POS_S)
    {
        if(controlStick.getRawButton(RobotInterface.SCROLL_HEIGHT)){
            driveForklift(controlStick, m_rightstick, PEG_POS_S, PEG_POS_F, m_liftVictor, liftPot);
        }else{
        desiredValue = getDesiredPotValue(controlStick, desiredValue, previousUp, previousDown);
        previousUp = controlStick.getRawButton(RobotInterface.FORK_UP);
        previousDown = controlStick.getRawButton(RobotInterface.FORK_DOWN);
        driveForklift(desiredValue, m_rightstick, PEG_POS_S, PEG_POS_F, m_liftVictor, liftPot);
        }

        loadSelectedArray(controlStick, PEG_POS_S, PEG_POS_F);
    }
        public boolean manageForklift(int pegHeight, Victor m_liftVictor, AnalogChannel liftPot)
    {
        atPeg = driveAutoLift(pegHeight, m_liftVictor, liftPot);
        return atPeg;
    }

            private boolean driveAutoLift(int pegPOS, Victor m_liftVictor, AnalogChannel liftPot)
    {
        if((pegPOS <= liftPot.getValue() &&
                RobotInterface.Max_Height <= liftPot.getValue()))
        {
            m_liftVictor.set(1);
            atPeg = false;
        }
        else
        {
            if((pegPOS >= liftPot.getValue() &&
                    RobotInterface.Min_Height >= liftPot.getValue()))
            {
                m_liftVictor.set(-1);
                atPeg = false;
            }
            else
            {
                m_liftVictor.set(0);
                atPeg = true;
            }
        }
        return atPeg;
    }
    public void loadSelectedArray(Joystick m_controller, int[] PEG_POS_S, int[] PEG_POS_F){
                if(m_controller.getRawButton(RobotInterface.Pole_Selector)){
                    for(int i=1; i<=4; i++){
                        potentiometerPositions[i] = PEG_POS_S[i];
                    }
                }else{
                    for(int i=1; i<=4; i++){
                        potentiometerPositions[i] = PEG_POS_F[i];
                    }
                }
            }

}
