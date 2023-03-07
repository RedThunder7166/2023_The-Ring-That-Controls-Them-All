// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceOnBeamCommand extends CommandBase {

  private  Swerve swerve;

  private double error;
  private double currentAngle;
  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public BalanceOnBeamCommand(Swerve swerve) {
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = swerve.getPitch();

    error = 0 - currentAngle;
    drivePower = -Math.min(5 * error, 1);
    
    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePower < 0) {
      drivePower *= 1.35;
    }

    // Limit the max power
    // if (Math.abs(drivePower) > 1) {
    //   drivePower = Math.copySign(1, drivePower);
    // }

    swerve.drive(new Translation2d(drivePower, 0), 0, false, false);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < 1; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}