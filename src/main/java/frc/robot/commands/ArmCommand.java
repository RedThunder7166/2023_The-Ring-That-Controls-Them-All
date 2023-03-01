// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.theCLAAAWWW;

public class ArmCommand extends CommandBase {
  private theCLAAAWWW clawSubsystem;
  private double targetAngle;
  private double initialAngle;
  private boolean byeFelicia = false;
  
  /** Creates a new ArmCommand. */
  public ArmCommand(theCLAAAWWW clawSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    targetAngle = angle;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = clawSubsystem.getArmAngle();
    clawSubsystem.setArmAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (initialAngle < targetAngle){
    //   if(clawSubsystem.getArmAngle() > targetAngle - 1){
    //     byeFelicia = !byeFelicia;
    //   }
    // }else{
    //   if(clawSubsystem.getArmAngle() < targetAngle + 1){
    //     byeFelicia = !byeFelicia;
    //   }

    // }
    if(Math.abs(clawSubsystem.getArmAngle() - targetAngle) < 2){
      byeFelicia = !byeFelicia;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    byeFelicia = !byeFelicia;
    // clawSubsystem.setArmAngle(targetAngle);
    clawSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (clawSubsystem.isArmSwitchPressed()) {
      return true;
    }
    return byeFelicia;
  }
}
