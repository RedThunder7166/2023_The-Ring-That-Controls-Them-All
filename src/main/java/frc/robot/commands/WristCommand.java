// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.theCLAAAWWW;

public class WristCommand extends CommandBase {
  private theCLAAAWWW clawSubsystem;
  private double targetAngle;
  private double initialAngle;
  private boolean byeFelicia = false;
  
  /** Creates a new WristCommand. */
  public WristCommand(theCLAAAWWW clawSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.clawSubsystem = clawSubsystem;
    targetAngle = angle;
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = clawSubsystem.getWristAngle();
    clawSubsystem.setWristAngle(targetAngle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("BEGIN EXECUTE METHOD");
    // if (initialAngle < targetAngle){
    //   System.out.println("" + initialAngle + " is less than " + targetAngle);
    //   if(clawSubsystem.getWristAngle() > targetAngle){
    //     System.out.println("" + clawSubsystem.getWristAngle() + " is greater than " + targetAngle);
    //     byeFelicia = !byeFelicia;
    //   }
    // }else{
    //   if(clawSubsystem.getWristAngle() < targetAngle){
    //     System.out.println("" + clawSubsystem.getWristAngle() + " is less than " + targetAngle);
    //     byeFelicia = !byeFelicia;
    //   }

    if((Math.abs(clawSubsystem.getWristAngle() - targetAngle) < 2)){
      byeFelicia = !byeFelicia;
    }
    //System.out.println("END EXECUTE METHOD");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    byeFelicia = !byeFelicia;
    clawSubsystem.setWristAngle(targetAngle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return byeFelicia;
  }
}
