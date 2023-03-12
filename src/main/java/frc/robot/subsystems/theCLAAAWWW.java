// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm;
import frc.robot.Wrist;
import frc.robot.Constants.Clawstants;

import java.lang.invoke.MethodHandles.Lookup.ClassOption;

import frc.lib.util.ClawUtils;

public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private Buttons m_Buttons = new Buttons();
  public enum ClawState {
  
    TRANSPORT, LOADING, LOW, MEDIUM, HIGH
  }

  public ClawState previousState;
  private double armAngle;


  Arm arm = Arm.getInstance();
  Wrist wrist = Wrist.getInstance();
  public ClawState clawState = getState();


  // TODO Dont forget that EVERY thing needs a god forbidden PID.

  private boolean isToggled = false;
  private double closedPos = 0;

  public theCLAAAWWW() {
    System.out.println("CONSTRUCTOR STATE: " + clawState.name());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Reset Encoder", arm.isArmSwitchPressed());
    if(arm.isArmSwitchPressed()){
      arm.zeroEncoder();
    }

    // if(clawState != clawState.LOADING && clawState != clawState.TRANSPORT){
    //   //System.out.println("getState: " + clawState.name());
    clawState = getState();
    // }y
    
    SmartDashboard.putString("ClawState", clawState.name());

    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
    SmartDashboard.putNumber("Arm Encoder", arm.getRawEncoderUnits());

    SmartDashboard.putNumber("Wrist Absolute", wrist.getWristAbsolute());
    SmartDashboard.putNumber("Wrist Angle", wrist.getWristAngle());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getRawEncoderUnits());
    
    SmartDashboard.getBoolean("Closed", isToggled);
  }

public void syncEncoders(){
  wrist.syncEncoders();
}

//*Create Claw close/open abilites */


private void toggleButton(){

  isToggled = !isToggled;
  
}

private void toggleCone(){

  closedPos = Clawstants.closedCone;
  

}

public void holdArm(){
  arm.holdAngle();
}

private void toggleCube(){

  closedPos = Clawstants.closedCube;

}

public void drive(double speed, double wristSpeed){
//  System.out.println( 0.1* (speed + Math.sin(arm.getAngle())) );
  arm.drive(speed * 0.25);
  wrist.driveWrist(wristSpeed * 0.25);

}

public void setArmAngle(double armAngle){
  arm.setAngle(armAngle);
}

public void setWristAngle(double wristAngle){
  wrist.setAngle(wristAngle);
}
 
public void stopWrist() {
  wrist.stop();
}
public void stopArm() {
  arm.stop();
}

public boolean areWristSwitchesPressed(){
  return wrist.isNegativeSwitchPressed() || wrist.isPositiveSwitchPressed();
}
public boolean isArmSwitchPressed(){
  return arm.isArmSwitchPressed();
}

public double getWristAngle(){
  return wrist.getWristAngle();
}

public double getArmAngle(){
  return arm.getAngle();
}

public ClawState getState(){

  if(arm.getAngle() <= 10 && wrist.getWristAbsolute() > Clawstants.wristMedium){
    return ClawState.LOADING;
  } 
  else if (arm.getAngle() <= 10 && wrist.getWristAbsolute() < Clawstants.wristMedium){
    return ClawState.TRANSPORT;
  } 
  else if (arm.getAngle() <= 43){
    return ClawState.LOW;
  }
  else if(arm.getAngle() > 43 && arm.getAngle() < 82.5){
    return ClawState.MEDIUM;
  }
  else if(arm.getAngle() >= 82.5){
    return ClawState.HIGH;
  }

  return null;
}

public void setState(ClawState newState){
  System.out.println("SETTING STATE TO newState: " + newState.name());
  clawState = newState;
}

// public void driveGripper(double speed) {
//   double kMax = 1000; // TODO change value to encoder values set via Tuner
//   double kMin = 100;
//   double kother = 10;
//   if (cancoder.getAbsolutePosition() > kMax && speed > 0) { // absolute value v. Posistion
//     gripperMotor.set(ControlMode.Position, kMin);
//   } else if (cancoder.getAbsolutePosition() < kMin && speed < 0) {
//     gripperMotor.set(ControlMode.Position, kMax);

//   } else {
//     gripperMotor.set(ControlMode.PercentOutput, speed);

//   }
// }




}