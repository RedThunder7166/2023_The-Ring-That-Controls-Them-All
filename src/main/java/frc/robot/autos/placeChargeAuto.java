// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class placeChargeAuto extends SequentialCommandGroup {
  /** Creates a new placeChargeAuto. */
  public placeChargeAuto(Swerve s_Swerve, theCLAAAWWW s_Claaawww, GripperSubsystem s_Gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // move backwards
      // new InstantCommand(() -> s_Swerve.drive(
      //   new Translation2d(-1, 0), 0, true, true), s_Swerve),
      new DriveMeters(s_Swerve, -1, 0, 0),
      // move the wrist to loading
      new WristCommand(s_Claaawww, Clawstants.wristLoading),
      // gripper grab the piece (commented out because it should already be gripped)
      // new GripperCommand(s_Gripper, Clawstants.closedCube),
      // move the wrist to grabbed
      new WristCommand(s_Claaawww, Clawstants.wristGrabbed),
      // move the arm to medium
      new ArmCommand(s_Claaawww, Clawstants.armMedium),
      // move the wrist to high
      new WristCommand(s_Claaawww, Clawstants.wristMedium),
      // gripper lets go of piece
      new GripperCommand(s_Gripper, Clawstants.openAll),

      // move the wrist to loading
      new WristCommand(s_Claaawww, Clawstants.wristLoading),
      // move the arm to loading
      new ArmCommand(s_Claaawww, Clawstants.armLoading)
    );
  }
}
