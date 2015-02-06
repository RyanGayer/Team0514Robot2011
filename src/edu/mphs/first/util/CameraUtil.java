/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.mphs.first.util;

import edu.mphs.first.interfaces.RobotInterface;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;


public class CameraUtil {
    boolean autoMode;
    double kScoreThreshold = .01;
    double previousX, previousY = .5;
    double pidOutput;

   public void CameraUitl(){

   }
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void cameraInit(AxisCamera cam, Gyro gyro, Joystick controller,
                            Servo servoX, Servo servoY, boolean autoMode) {
//        Timer.delay(10.0);
        Watchdog.getInstance().setExpiration(.75);
        cam.writeResolution(AxisCamera.ResolutionT.k160x120);
        //cam.writeColorLevel(1);
        cam.writeBrightness(50);
        cam.writeCompression(25);
        gyro.setSensitivity(.007);
        centerCamera(controller, servoX, servoY, autoMode);
  }

    /**
     * This function is called periodically during operator control
     */
    public double manageCamera(boolean autoMode, AxisCamera cam, Gyro gyro, RobotDrive drive,
                             Joystick controller,
                             Servo servoX, Servo servoY,
                             DashboardUtil trackerDashboard) {
        Watchdog.getInstance().feed();

        if (autoMode){
            pidOutput = autoCamera(cam, gyro, trackerDashboard);
        }else{
            teleopCamera(cam, controller, servoX, servoY, autoMode);
            pidOutput = 0;
        }
        if (!autoMode){
            /*
             * Don't run the true up calculation - too CPU intensive!
            if(controller.getRawButton(RobotInterface.TRUE_UP)){
                centerCamera(controller, servoX, servoY, true);
                pidOutput = autoCamera(cam, gyro, trackerDashboard);
            }
             */
//            if(controller.getRawButton(RobotInterface.CENTER_CAM)){
//                centerCamera(controller, servoX, servoY, autoMode);
//            }
        }
        return pidOutput;
    }

    private void teleopCamera(AxisCamera cam, Joystick controller, Servo servoX, Servo servoY, boolean autoMode){
//        double startTime = Timer.getFPGATimestamp();

       panControll(controller, servoX, servoY);
       centerCamera(controller, servoX, servoY, autoMode);


           ColorImage image = null;
            try {
                Watchdog.getInstance().feed();

                if (cam.freshImage()) {
                    image = cam.getImage();
//                    trackerDashboard.updateVisionDashboard(0.0, targets[0].getHorizontalAngle(), 0.0, 0.0, newTargets);
                    Thread.yield();
                }
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            } catch (AxisCameraException ex) {
                ex.printStackTrace();
            } finally {
//                System.out.println("Finally");
                try {
                    Watchdog.getInstance().feed();

                    if (image != null) {
//                        System.out.println("Image Free");
                        image.free();
                    }else{
 //                       System.out.println("Image Null");
                    }

                } catch (NIVisionException ex) {
                    System.out.println("NIVisionException thrown...");
                    ex.printStackTrace();
                }
            }
    }
    
  private double autoCamera(AxisCamera cam, Gyro gyro, DashboardUtil trackerDashboard){
       double pidDouble = 0;
//       double startTime = Timer.getFPGATimestamp();

           ColorImage image = null;
            try {
                Watchdog.getInstance().feed();
                if (cam.freshImage()){ //&& turnController.onTarget()) {
                    double gyroAngle = gyro.pidGet();
                    image = cam.getImage();
                    Thread.yield();
                    TargetUtil[] targets = TargetUtil.findCircularTargets(image);
                    Thread.yield();
                    if (targets.length == 0 || targets[0].m_score < kScoreThreshold) {
  //                      System.out.println("No target found");
                        TargetUtil[] newTargets = new TargetUtil[targets.length + 1];
                        newTargets[0] = new TargetUtil();
                        newTargets[0].m_majorRadius = 0;
                        newTargets[0].m_minorRadius = 0;
                        newTargets[0].m_score = 0;
                        for (int i = 0; i < targets.length; i++) {
                            newTargets[i + 1] = targets[i];
                        }
                        trackerDashboard.updateVisionDashboard(0.0, gyro.getAngle(), 0.0, 0.0, newTargets);
                    } else {
//                        System.out.println(targets[0]);
//                        System.out.println("Target Angle: " + targets[0].getHorizontalAngle());
//                        turnController.setSetpoint(gyroAngle + targets[0].getHorizontalAngle());
                        pidDouble = (gyroAngle + targets[0].getHorizontalAngle());
                        trackerDashboard.updateVisionDashboard(0.0, gyro.getAngle(), 0.0, targets[0].m_xPos / targets[0].m_xMax, targets);
                    }
                }
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            } catch (AxisCameraException ex) {
                ex.printStackTrace();
            } finally {
//                System.out.println("Finally");
                try {
                    if (image != null) {
//                        System.out.println("Image Free");
                        image.free();
                    }else{
//                        System.out.println("Image Null");
                    }

                } catch (NIVisionException ex) {
                    System.out.println("NIVisionException thrown...");
                    ex.printStackTrace();
                }
            }
            return pidDouble;
        }
        private double doubleRound(double unrounded){
        if(unrounded > 0)
        {
            return 1;
        }
        else
        {
            if(unrounded < -.1)
            {
                return 0;
            }
            else
            {
                return .5;
            }
        }
    }
        public void panControll(Joystick controller, Servo servoX, Servo servoY){
       if(doubleRound(controller.getRawAxis(RobotInterface.dPAD_PAN_Y)) == 1)
       {
           servoY.set(previousY - .01);
           previousY = previousY - .01;
       }
       else
       {
           if(doubleRound(controller.getRawAxis(RobotInterface.dPAD_PAN_Y)) == 0)
           {
               servoY.set(previousY + .01);
               previousY = previousY + .01;
           }
           else
           {
               servoY.set(previousY);
           }
       }

       if(doubleRound(controller.getRawAxis(RobotInterface.dPAD_PAN_X)) == 1)
       {
           servoX.set(previousX + .01);
           previousX = previousX + .01;
       }
       else
       {
           if(doubleRound(controller.getRawAxis(RobotInterface.dPAD_PAN_X)) == 0)
           {
               servoX.set(previousX - .01);
               previousX = previousX - .01;
           }
           else
           {
               servoX.set(previousX);
           }
        }
       if(previousY > 1)
       {
           previousY = 1;
       }
       if(previousY < 0)
       {
           previousY = 0;
       }
       if(previousX > 1)
       {
           previousX = 1;
       }
       if(previousX < 0)
       {
           previousX = 0;
       }

  }
        public void centerCamera(Joystick controller, Servo servoX, Servo servoY, boolean autoMode){
           if(autoMode){
               servoX.set(.5);
               previousY = .5;
               servoY.set(.5);
               previousX = .5;
           }else{
/*
                if(controller.getRawButton(RobotInterface.CENTER_CAM)){
                   servoX.set(.5);
                   previousY = .5;
                   servoY.set(.5);
                   previousX = .5;
               }
 *
 */
           }

   }

}
