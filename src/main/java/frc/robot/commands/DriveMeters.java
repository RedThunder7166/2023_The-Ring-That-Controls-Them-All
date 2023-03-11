// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveMeters extends CommandBase {
  /** Creates a new DriveMeters. */
  private Swerve s_Swerve;
  private double targetMetersX;
  private double targetMetersY;
  private double targetDegrees;
  private Pose2d currentPose;

  private double x_error;
  private double y_error;
  private double Deg_error;
  private boolean reachedRotation;
  // private double feedforward = 1;
  public DriveMeters(Swerve swerve, double targetX, double targetY, double targetDeg) {
    s_Swerve = swerve;
    targetMetersX = targetX;
    targetMetersY = targetY;
    targetDegrees = targetDeg;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Dont know why this fixed t but it did
    x_error = 0;
    y_error = 0;
    Deg_error = 0;
    reachedRotation = false;

    s_Swerve.resetOdometry(new Pose2d(0,0, s_Swerve.getYaw()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = s_Swerve.getPose();
    x_error = targetMetersX - currentPose.getX();
    y_error = targetMetersY - currentPose.getY();
    Deg_error = targetDegrees - currentPose.getRotation().getDegrees();
    // System.out.println(x_error);
    
    // System.out.println(y_error);
    // System.out.println(rot_error);

    double x_kP = 4;
    double y_kP = 4;
    double Deg_kP = 4;

    double x = x_kP * x_error                                                                             ;
    double y = y_kP * y_error;
    double Deg = Deg_kP * Deg_error;

    if (Math.abs(currentPose.getRotation().getDegrees() - Math.abs(targetDegrees)) <= 1) {
      reachedRotation = true;
      Deg = targetDegrees;
    }
    s_Swerve.drive(
        new Translation2d(x, y),
        Math.toRadians(Deg),
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
        new Translation2d(0, 0),
        0,
        true,
        false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("CURRENT DEGREES", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("TARGET ROTATION", targetDegrees);
    if (Math.abs(currentPose.getX()) >= Math.abs(targetMetersX) 
    && Math.abs(currentPose.getY()) >= Math.abs(targetMetersY) 
    && reachedRotation)
    {
      return true;
    }
    return false;
  }
}
