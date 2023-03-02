// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Clawstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveMeters;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class placeDriveAuto extends SequentialCommandGroup {
  /** Creates a new autoCommandGroup. */

  public placeDriveAuto(Swerve s_Swerve, theCLAAAWWW clawSubsystem, GripperSubsystem s_Gripper) {

    addCommands(
      // move backwards
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(-1, 0), 0, true, true), s_Swerve),
      new DriveMeters(s_Swerve, -1, 0, 0),

      // do not do the below code because we already are gripping a piece
      // move the wrist to loading
      // new WristCommand(clawSubsystem, Clawstants.wristLoading),
      // gripper grab the piece

      // move the wrist to grabbed
      new WristCommand(clawSubsystem, Clawstants.wristGrabbed),
      // move the arm to medium
      new ArmCommand(clawSubsystem, Clawstants.armMedium),
      // move the wrist to high
      new WristCommand(clawSubsystem, Clawstants.wristHigh),

      // move the arm to high
      new ArmCommand(clawSubsystem, Clawstants.armHigh),

      // move forwards
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(1, -0), 0, true, true), s_Swerve),
      new DriveMeters(s_Swerve, 1, 0, 0),
      // gripper lets go of piece
      new GripperCommand(s_Gripper, Clawstants.openAll),
      // move backwards
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(-1, 0), 0, true, true), s_Swerve),
      new DriveMeters(s_Swerve, -1, 0, 0),
      // move the arm to medium
      new ArmCommand(clawSubsystem, Clawstants.armMedium),
      // move the wrist to loading
      new WristCommand(clawSubsystem, Clawstants.wristLoading),
      // move the arm to loading
      new ArmCommand(clawSubsystem, Clawstants.armLoading),

      // do a 180 AND
      // go to the piece on the ground
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(-3, 0), 180, true, true), s_Swerve)
      new DriveMeters(s_Swerve, -3, 0, 0)
    );
  }
}
