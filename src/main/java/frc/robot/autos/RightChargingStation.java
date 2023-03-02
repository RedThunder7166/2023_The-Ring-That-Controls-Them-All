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
import frc.robot.commands.ClawCommand;
import frc.robot.commands.CloseGripperCommand;
import frc.robot.commands.DriveMeters;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightChargingStation extends SequentialCommandGroup {
  /** Creates a new ChargingStation. */
  public RightChargingStation(Swerve s_Swerve) {
  
    addCommands(
      // move backwards
      // new PrintCommand("Starting Auto"),
      // new DriveMeters(s_Swerve, -1, 0, 0),
      // new PrintCommand("Ending Auto"),
      // new DriveMeters(s_Swerve, 1, 0, 0)

      new PrintCommand("Starting ChargingAuto"),
      new DriveMeters(s_Swerve, 4, 0, 0),
      new PrintCommand("move right now"),
      new DriveMeters(s_Swerve, 0, 1.5, 0),//i dont remember if POS or NEG is left or right so currently just recoginze that it needs to be changed
      new DriveMeters(s_Swerve, -1.5, 0, 0),
      new DriveMeters(s_Swerve, 0, 0 , .01)//used to turn the wheels in preperation for slipping

    );
  }
}