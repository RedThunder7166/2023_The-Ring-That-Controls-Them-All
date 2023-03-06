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
import frc.robot.commands.YfeedBackwardAuto;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAutoLeft extends SequentialCommandGroup {
  /** Creates a new autoCommandGroup. */

  public PlaceAutoLeft(Swerve s_Swerve, theCLAAAWWW clawSubsystem, GripperSubsystem s_Gripper) {

    addCommands(
      new PrintCommand("Starting TestAuto"),
      //new CloseGripperCommand(s_Gripper, Clawstants.closedCube),
      //new WristCommand(clawSubsystem, Clawstants.wristGrabbed),
      //new ArmCommand(clawSubsystem, Clawstants.armMedium),
     // new WristCommand(clawSubsystem, Clawstants.wristMedium),
     // new OpenGripperCommand(s_Gripper, Clawstants.openAll),
      
     //new WristCommand(clawSubsystem, Clawstants.wristTransport),
      //new ArmCommand(clawSubsystem, Clawstants.armLoading),

      new YfeedBackwardAuto(s_Swerve, 0, -0.3, 0),
      new DriveMeters(s_Swerve, -5, 0, 0)
    );
  }
}
