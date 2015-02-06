/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.mphs.first.util;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 * @author Developer
 */
public class DeployUtil {
    boolean deployFlag;
    public DeployUtil()
    {
        deployFlag = false;
    }

    public void manageDeploy(Joystick controller, DoubleSolenoid deployCylinder){
        if(controller.getRawButton(RobotInterface.DEPLOY_BOT))
        {
            deployFlag = true;
        }
        if(controller.getRawButton(RobotInterface.RETURN_BOT)){
            deployFlag = false;
        }
        if(deployFlag){
            deployCylinder.set(DoubleSolenoid.Value.kForward);
        }else{
            deployCylinder.set(DoubleSolenoid.Value.kReverse);
        }

    }
}
