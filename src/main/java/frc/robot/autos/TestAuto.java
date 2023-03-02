// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Clawstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveMeters;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new autoCommandGroup. */

  public TestAuto(Swerve s_Swerve, theCLAAAWWW clawSubsystem) {

    addCommands(
      // move backwards
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(-1, 0), 0, true, true), s_Swerve),
      new PrintCommand("Starting Auto"),
      new DriveMeters(s_Swerve, -1, 0, 0),
      new PrintCommand("Ending Auto"),
      new DriveMeters(s_Swerve, 1, 0, 0)
    );
  }
}
