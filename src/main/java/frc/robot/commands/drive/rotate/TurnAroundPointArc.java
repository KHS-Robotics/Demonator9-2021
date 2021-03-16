// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class TurnAroundPointArc extends TurnAroundPoint {
  /** Creates a new TurnAroundPointArc. 
   * @param centerPoint the center point of the rotation
   * @param clockwise drive clockwise around the point
   * @param finalRot Between -180 and 180.
  */
  public TurnAroundPointArc(Translation2d centerPoint, boolean clockwise, double finalRot) {
    super(centerPoint, clockwise);

    super.finalRot = finalRot;
  }
}
