// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Arm;
import frc.robot.Wrist;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

public class theCLAAAWWW extends SubsystemBase {
  /** Creates a new theCLAAAWWW. */
  private Buttons m_Buttons = new Buttons();

  public enum ClawState {
  
    LOADING, LOW, MEDIUM, HIGH
  }

  ClawState clawState;
  ClawState previousState;
  Arm arm = Arm.getInstance();
  Wrist wrist = Wrist.getInstance();

  // TODO Dont forget that EVERY thing needs a god forbidden PID.

  private boolean isToggled = false;


  // ArmFeedforward armFeedForward = new ArmFeedforward(.21469, 0.43923, 2.0662);

  public theCLAAAWWW() {

    clawState = ClawState.LOADING;



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    previousState = clawState;

    if (m_Buttons.isPressed(1)) {
      clawState = ClawState.LOADING;
    } else if (m_Buttons.isPressed(2)) {
      clawState = ClawState.LOW;
    }
      else if (m_Buttons.isPressed(3)){
        clawState = ClawState.HIGH;
      }

    //Check to see if state has changed before moving the arm or claw.
    if (clawState != previousState) {
      switch (clawState) {
        case LOADING:
          wrist.setAngle(0);
          arm.setAngle(0);
          break;
        case LOW:
          wrist.setAngle(0);
          arm.setAngle(30);
          break;
        case HIGH:
        wrist.setAngle(30);
        arm.setAngle(45);
          break;
        case MEDIUM:
          break;
      }

    }

    // IMPORTANT - DO NOT DELETE
    // setArmMotorsAngle(armAngle);
    
    SmartDashboard.putString("ClawState", clawState.name());

    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
    SmartDashboard.putNumber("Wrist Angle", arm.getAngle());


    SmartDashboard.putNumber("Arm Encoder", arm.getRawEncoderUnits());
    SmartDashboard.putNumber("Wrist Encoder", wrist.getRawEncoderUnits());

  }
/* Button things */

public void setClawstate(ClawState CS) {

  clawState = CS;
}

//*Create Claw close/open abilites */
CANCoder cancoder = new CANCoder(68);

VictorSPX gripperMotor = new VictorSPX(69);

private void toggleButton(){

  isToggled = !isToggled;

}

private void toggleCone(){

  isToggled = !isToggled;

}


private void toggleCube(){

  isToggled = !isToggled;

}





public void driveGripper(double speed) {
  double kMax = 1000; // TODO change value to encoder values set via Tuner
  double kMin = 100;
  if (cancoder.getAbsolutePosition() > kMax && speed > 0) { // absolute value v. Posistion
    gripperMotor.set(ControlMode.Position, kMin);
  } else if (cancoder.getAbsolutePosition() < kMin && speed < 0) {
    gripperMotor.set(ControlMode.Position, kMax);

  } else {
    gripperMotor.set(ControlMode.PercentOutput, speed);

  }
}




}