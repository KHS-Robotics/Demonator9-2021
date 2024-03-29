// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.RobotContainer;

public class TurnAroundRelativePoint extends TurnAroundPoint {
  /** Creates a new TurnAroundRelativePoint. */
  public TurnAroundRelativePoint(Translation2d relativePoint, boolean clockwise) {
    super(new Translation2d( RobotContainer.swerveDrive.getPose().getX() + relativePoint.getX(), RobotContainer.swerveDrive.getPose().getY() + relativePoint.getY() ), clockwise);
  }
}
