// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class GalacticSearch {
	// 4 expected pos/scales here
	static Vector2d[] A1Pos = { new Vector2d(0, 0) };
	static Vector2d[] A1Scale = { new Vector2d(0, 0) };

	static Vector2d[] A2Pos = { new Vector2d(0, 0) };
	static Vector2d[] A2Scale = { new Vector2d(0, 0) };

	static Vector2d[] B1Pos = { new Vector2d(0, 0) };
	static Vector2d[] B1Scale = { new Vector2d(0, 0) };

	static Vector2d[] B2Pos = { new Vector2d(0, 0) };
	static Vector2d[] B2Scale = { new Vector2d(0, 0) };

	// compare array lengths, only check with those which are equal

	public static int CalculatePath(Vector2d[] positions, Vector2d[] scaling) {
		if (positions.length == 2) {

		} else if (positions.length == 3) {

		}

		return 0;
	}
}
