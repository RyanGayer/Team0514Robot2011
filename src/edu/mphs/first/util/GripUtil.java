/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author Developer
 */
public class GripUtil {
boolean wristToggle, gripToggle;
boolean pegDeployed;

    public GripUtil()
    {
        wristToggle = false;
        gripToggle = false;
        pegDeployed = false;
    }

    public void manageGrip(Solenoid wristSolenoid, Solenoid gripSolenoid, DoubleSolenoid armSolenoid,
                           Joystick controller)
    {
        if(controller.getRawButton(RobotInterface.ARM_OUT)){
            armSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        if(controller.getRawButton(RobotInterface.ARM_RETURN)){
            armSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        if(controller.getRawButton(RobotInterface.GRIP) && !gripToggle)
        {
            gripSolenoid.set(!gripSolenoid.get());
            System.out.println("Grip Solenoid is now " + gripSolenoid.get());
        }
        gripToggle = controller.getRawButton(RobotInterface.GRIP);

        if(controller.getRawButton(RobotInterface.WRIST) && !wristToggle)
        {
            wristSolenoid.set(!wristSolenoid.get());
            System.out.println("Wrist Solenoid is now " + wristSolenoid.get());
        }
        wristToggle = controller.getRawButton(RobotInterface.WRIST);
    }
        public boolean manageAutoGrip(int m_autoStep, Solenoid wristSolenoid, Solenoid gripSolenoid,
                                      DoubleSolenoid armSolenoid, Victor m_liftVictor, int loopCounter,
                                      int processSwitch){
                if(processSwitch == RobotInterface.PICK_UP){
                switch (loopCounter){
                    case 100:
                        gripSolenoid.set(false);                        
                    break;    
                    case 150:
                        //Stop the lift, drop the wrist and return the are back.
                        //m_liftVictor.set(0);
                        wristSolenoid.set(true);
                        pegDeployed = true;
                    break;
                    default:
                        pegDeployed = false;
                    break;
                }
            }
            if(processSwitch == RobotInterface.HANG_UP){
                switch (loopCounter){
                    case 100:
                        //Release the grip on the tub.
                        gripSolenoid.set(true);
                    break;
                    case 150:
                        //Stop the lift, drop the wrist and return the are back.
                        //m_liftVictor.set(0);
                        wristSolenoid.set(false);
                        //armSolenoid.set(DoubleSolenoid.Value.kReverse);
                        pegDeployed = true;
                    break;
                    default:
                        pegDeployed = false;
                    break;
                }                
            }
            return pegDeployed;       
    }
}
