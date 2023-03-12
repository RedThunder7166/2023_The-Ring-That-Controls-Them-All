// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import javax.naming.PartialResultException;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Clawstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmWristCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DriveMeters;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.YfeedBackwardAuto;
import frc.robot.commands.YfeedforwardAuto;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.theCLAAAWWW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPieceAuto. */
  public TwoPieceAuto(Swerve s_Swerve, theCLAAAWWW s_Claw, GripperSubsystem s_Grip) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       // Step 1
            new ArmCommand(s_Claw, Clawstants.armHigh),
            new WristCommand(s_Claw, Clawstants.wristLow),
        new DriveMeters(s_Swerve, 1, 0, 0), // step 2
       // new GripperCommand(s_Grip, Clawstants.openAll), // step 3
        new DriveMeters(s_Swerve, -1, 0, 0),
        new YfeedforwardAuto(s_Swerve, 0, 1, 0), // step 4 left or right i dont remember
        new ParallelCommandGroup(// step 5
            new DriveMeters(s_Swerve, -1.8, 0, 0),
            new ArmWristCommand(s_Claw, Clawstants.wristGrabbed, Clawstants.armTransport)),
        new DriveMeters(s_Swerve, -3.81, 0, -180), // step 6
     //   new GripperCommand(s_Grip, Clawstants.closedCone), // step 7
        new ParallelCommandGroup(// step 8
            new DriveMeters(s_Swerve, 2.45, .23, 180),
            new ArmCommand(s_Claw, Clawstants.armHigh)),
        new DriveMeters(s_Swerve, 2.3, 0, 0)); // step 9
     //   new GripperCommand(s_Grip, Clawstants.openAll));// step 10
  }
}
