// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class PathPlaaanner extends SubsystemBase {
  /** Creates a new PathPlanner. */
  private Swerve s_Swerve;
  private SwerveAutoBuilder m_AutoBuilder;

  public PathPlaaanner(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Lower Claw", new PrintCommand("LOWERING CLAW"));
    eventMap.put("event", new PrintCommand("VENTING"));

    // Create the AutoBuilder. This only needs to be created once when robot code
    // starts, not every time you want to create an auto command. A good place to
    // put this is in RobotContainer along with your subsystems.
    m_AutoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(1, 0.0, 0), // PID constants to correct for translation error (used to create the X and Y
                                     // PID controllers)
        new PIDConstants(.5, 0.0, 0), // PID constants to correct for rotation error (used to create the rotation
                                      // controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color.
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following
    );
  }
  
  @Override
  public void periodic() {
    Pose2d pose = s_Swerve.getPose();
    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());
    SmartDashboard.putNumber("Odometry Rot", pose.getRotation().getRadians());
  }
  
  public SequentialCommandGroup newFullAuto(PathPlannerTrajectory path) {
    return new SequentialCommandGroup(
      new InstantCommand(()-> s_Swerve.resetOdometry(new Pose2d(0,0, s_Swerve.getYaw()))),
      m_AutoBuilder.fullAuto(path),
      new PrintCommand("goodbye")

    );
  }
}
