/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSwerveWithXbox extends CommandBase {
  private boolean fieldRelative = false;

  public DriveSwerveWithXbox() {
    this.addRequirements(RobotContainer.swerveDrive);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() { // Order 66

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getY(GenericHID.Hand.kLeft)) * SwerveDrive.kMaxSpeed;
    if(Math.abs(xSpeed) < 0.05) {
      xSpeed = 0;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.kLeft)) * SwerveDrive.kMaxSpeed;
    if(Math.abs(ySpeed) < 0.05) {
      ySpeed = 0;
    }

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.

    //Deadband on new controller
    double rot = 0;
    if(Math.abs(RobotContainer.xboxController.getX(GenericHID.Hand.kRight)) > 0.05) {
      rot = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getX(GenericHID.Hand.kRight)) * SwerveDrive.kMaxAngularSpeed;
    }
    
    fieldRelative = (!RobotContainer.xboxController.getBumper(GenericHID.Hand.kLeft));
    RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative);

    /*
     * if (Math.abs(x) + Math.abs(y) + Math.abs(z) > 0.35) {
     * Robot.swerveDrive.disablePID(); }
     */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
  }
}