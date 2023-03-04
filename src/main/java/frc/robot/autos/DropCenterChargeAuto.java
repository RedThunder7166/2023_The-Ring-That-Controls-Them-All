// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Clawstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.CloseGripperCommand;
import frc.robot.commands.DriveMeters;
import frc.robot.commands.OpenGripperCommand;
import frc.robot.commands.RampMeters;
import frc.robot.commands.RampMetersBack;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropCenterChargeAuto extends SequentialCommandGroup {
  /** Creates a new DropCenterChargeAuto. */
  public DropCenterChargeAuto(Swerve s_Swerve, theCLAAAWWW S_Claw, GripperSubsystem s_Gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // grip, move arm out, drop
      new CloseGripperCommand(s_Gripper, Clawstants.closedCube),
      new WristCommand(S_Claw, Clawstants.wristGrabbed),
      new ArmCommand(S_Claw, Clawstants.armBetweenLowAndMedium),
      //new WristCommand(S_Claw, Clawstants.wristLow),
      new OpenGripperCommand(s_Gripper, Clawstants.openAll),

      // // move arm & wrist back
      new ArmCommand(S_Claw, Clawstants.armLoading),
      //new WristCommand(S_Claw, Clawstants.wristLoading),

      // leave community ???
      new RampMetersBack(s_Swerve, -4.5, -.1, 0),
      // drive on charging station
      new RampMeters(s_Swerve, 2.45, 0, 0),
      new DriveMeters(s_Swerve, 0, 0, 1),
      new InstantCommand(()-> s_Swerve.drive(new Translation2d(), (.03), true, false))//used to turn the wheels in preperation for slipping
    );
  }
}
