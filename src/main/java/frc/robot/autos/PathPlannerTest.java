// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerTest extends SequentialCommandGroup {
  /** Creates a new PathPlannerTest. */
  public PathPlannerTest(Swerve m_Swerve, SwerveAutoBuilder m_AutoBuilder) {
    // PathPlannerTrajectory traj = PathPlanner.loadPath("thing", new PathConstraints(3, 5));
    PathPlannerTrajectory path = PathPlanner.loadPath("thing", new PathConstraints(3, 5));

    addCommands(
      // new BalanceOnBeamCommand(m_Swerve)
      // m_Swerve.followTrajectoryCommand(traj, false)
      m_AutoBuilder.fullAuto(path)
    );
  }
}
