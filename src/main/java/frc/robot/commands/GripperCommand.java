// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
  /** Creates a new GripperCommand. */
  private final GripperSubsystem m_GripperSubsystem;
  private double targetValue;
  private boolean isClosing;
  private double currPos;

  public GripperCommand(GripperSubsystem gripperSub, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperSub);
    m_GripperSubsystem = gripperSub;
    targetValue = target;

    currPos = m_GripperSubsystem.getPosition();
    

    if (currPos <= targetValue) {
      isClosing = false;
    }else{
      isClosing = true;
    }

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double direction = -0.9;//swapped negatives
    if (isClosing) {
      direction = 0.9;
    }
    m_GripperSubsystem.driveGripper(direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPos = m_GripperSubsystem.getPosition();
    SmartDashboard.putNumber("Gripper position", currPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_GripperSubsystem.driveGripper(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isClosing) {
      if (currPos <= targetValue) {
        return true;
      }
    } else {
      if (currPos >= targetValue) {
        return true;
      }
    }
    return false;
  }
}
