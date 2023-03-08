//We totally made this ourselves and definitly didn't steeeeeal this from the Ri3d peeps, we would neer do somethiung like that.
package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
// This command self=balances on the charging station using gyroscope pitch as feedback
public class GyroAutoBalance extends CommandBase {

  private  Swerve swerve;

  private double currentAngle;
  private double drivePower;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public GyroAutoBalance(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
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

    drivePower = Math.min(currentAngle, 0.25);
    // drivePower = copysign(drivePower, currentAngle);

    swerve.drive(new Translation2d(drivePower, 0), 0, true, true);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    // System.out.println("Error " + error);
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
    // return Math.abs(error) < 1; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    return currentAngle < 12;
  }
}