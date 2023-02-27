// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ClawUtils;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {
  private GripperSubsystem GripperSubsystem;
  private double targetAngle;
  private double initialAngle;
  private boolean helloFelicia = false;
  
  /** Creates a new WristCommand. */
  public GripperCommand(GripperSubsystem GripperSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.GripperSubsystem = GripperSubsystem;
    targetAngle = angle;
    addRequirements(GripperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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

   // if((Math.abs(.getGripperAngle() - targetAngle) < 2)){
      helloFelicia = !helloFelicia;
    }
    //System.out.println("END EXECUTE METHOD");

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   helloFelicia = !helloFelicia;
  //  GripperSubsystem.setWristAngle(targetAngle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return helloFelicia;
  }
}
