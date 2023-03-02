// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class OpenGripperCommand extends CommandBase {
  /** Creates a new OpenGripperCommand. */
  private final GripperSubsystem s_Gripper;
  private double targetValue;
  private double currentValue;
  public OpenGripperCommand(GripperSubsystem gripper, double value) {
    s_Gripper = gripper;
    addRequirements(s_Gripper);
    
    targetValue = value;
    // Use addRequirements() here to declare subsystem dependencies.

    currentValue = s_Gripper.getPosition();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentValue = s_Gripper.getPosition();
    s_Gripper.driveGripper(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Gripper.driveGripper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentValue >= targetValue) {
      return true;
    }
    return false;
  }
}
