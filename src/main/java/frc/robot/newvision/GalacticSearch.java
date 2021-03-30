// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.newvision;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class GalacticSearch {
	// 4 expected pos/scales here
	static Vector2d[] A1Pos = { new Vector2d(0, 0) };
	static Vector2d[] A1Scale = { new Vector2d(0, 0) };
	//[165.0,  149.0,  58.0,  30.0][226.0,  145.0,  16.0,  12.0][19.0,  131.0,  14.0,  9.0]

	static Vector2d[] A2Pos = { new Vector2d(0, 0) };
	static Vector2d[] A2Scale = { new Vector2d(0, 0) };
	//[269.0,  145.0,  14.0,  10.0][96.0,  134.0,  12.0,  5.0]
	//[272.0,  147.0,  16.0,  9.0][147.0,  136.0,  14.0,  5.0][95.0,  134.0,  6.0,  4.0]

	static Vector2d[] B1Pos = { new Vector2d(0, 0) };
	static Vector2d[] B1Scale = { new Vector2d(0, 0) };
	//[227.0,  146.0,  18.0,  17.0][95.0,  134.0,  10.0,  6.0]

	static Vector2d[] B2Pos = { new Vector2d(0, 0) };
	static Vector2d[] B2Scale = { new Vector2d(0, 0) };
	//[16.0,  135.0,  28.0,  22.0] [243.0,  125.0,  26.0,  20.0] [217.0,  80.0,  30.0,  11.0]

	// compare array lengths, only check with those which are equal

	public static int CalculatePath(Vector2d[] positions, Vector2d[] scaling) {
		if (positions.length == 2) {

		} else if (positions.length == 3) {

		}

		return 0;
	}
}
