// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.theCLAAAWWW;

public class ArmWristCommand extends CommandBase {
  /** Creates a new ArmWristCommand. */
  private theCLAAAWWW m_Claw;
  private double targetWristAngle;
  private double targetArmAngle;
  
  private double currentWristAngle;
  private double currentArmAngle;

  public ArmWristCommand(theCLAAAWWW claw, double targetWrist, double targetArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
    m_Claw = claw;
    targetWristAngle = targetWrist;
    targetArmAngle = targetArm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Claw.setWristAngle(targetWristAngle);
    m_Claw.setArmAngle(targetArmAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentWristAngle = m_Claw.getWristAngle();
    currentArmAngle = m_Claw.getArmAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Claw.stopWrist();
    m_Claw.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean wristFinished = false;
    boolean armFinished = false;
    if (m_Claw.areWristSwitchesPressed() || (Math.abs(currentWristAngle - targetWristAngle) < 1)) {
      m_Claw.stopWrist();
      wristFinished = true;
    }
    if (Math.abs(currentArmAngle - targetArmAngle) < 2){
      m_Claw.stopArm();
      armFinished = true;
    }
    return wristFinished && armFinished;
  }
}
