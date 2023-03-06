// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ConcurrentModificationException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class GyroBalance extends CommandBase {
  /** Creates a new GyroBalance. */

  public Swerve swerveSubsystem;
  private double output;
  private double maxSpeed;

  public GyroBalance(Swerve swerve, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.maxSpeed = maxSpeed;
    swerveSubsystem = swerve;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = swerveSubsystem.getPitch();
    output = Math.sin(currentAngle) * maxSpeed;
    swerveSubsystem.drive(new Translation2d(output, 0), 0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
