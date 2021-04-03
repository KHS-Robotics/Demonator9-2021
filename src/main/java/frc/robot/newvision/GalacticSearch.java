// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.newvision;

import edu.wpi.first.wpilibj.drive.Vector2d;

/** Add your docs here. */
public class GalacticSearch {
	// 4 expected pos/scales here
	static Vector2d[] A1Pos = { new Vector2d(165.0, 149.0), new Vector2d(226.0, 145.0), new Vector2d(19.0, 131.0) };
	static Vector2d[] A1Scale = { new Vector2d(58.0, 30.0), new Vector2d(16.0, 12.0), new Vector2d(14.0, 9.0) };
	// [165.0, 149.0, 58.0, 30.0][226.0, 145.0, 16.0, 12.0][19.0, 131.0, 14.0, 9.0]

	static Vector2d[] A2PosA = { new Vector2d(269.0, 145.0), new Vector2d(96.0, 134.0) };
	static Vector2d[] A2ScaleA = { new Vector2d(14.0, 10.0), new Vector2d(12.0, 5.0) };
	// [269.0, 145.0, 14.0, 10.0][96.0, 134.0, 12.0, 5.0]

	static Vector2d[] A2PosB = { new Vector2d(272.0, 147.0), new Vector2d(147.0, 136.0), new Vector2d(95.0, 134.0) };
	static Vector2d[] A2ScaleB = { new Vector2d(16.0, 9.0), new Vector2d(14.0, 5.0), new Vector2d(6.0, 4.0) };
	// [272.0, 147.0, 16.0, 9.0][147.0, 136.0, 14.0, 5.0][95.0, 134.0, 6.0, 4.0]

	static Vector2d[] B1Pos = { new Vector2d(227.0, 146.0), new Vector2d(95.0, 134.0) };
	static Vector2d[] B1Scale = { new Vector2d(18.0, 17.0), new Vector2d(10.0, 6.0) };
	// [227.0, 146.0, 18.0, 17.0][95.0, 134.0, 10.0, 6.0]

	static Vector2d[] B2Pos = { new Vector2d(16.0, 135.0), new Vector2d(243.0, 125.0), new Vector2d(217.0, 80.0) };
	static Vector2d[] B2Scale = { new Vector2d(28.0, 22.0), new Vector2d(26.0, 20.0), new Vector2d(30.0, 11.0) };
	// [16.0,  135.0,  28.0,  22.0] [243.0,  125.0,  26.0,  20.0] [217.0,  80.0,  30.0,  11.0]

	// compare array lengths, only check with those which are equal

	public static int CalculatePath(Vector2d[] positions, Vector2d[] scaling) {
		if (positions.length == 2) {
			double A2 = calculateDifference(positions, A2PosB);
			A2 += calculateDifference(scaling, A2ScaleB);
			
			double B1 = calculateDifference(positions, B1Pos);
			B1 += calculateDifference(scaling, B1Scale);

			if (A2 < B1) {
				return 1;
			} else {
				return 2;
			}

		} else if (positions.length == 3) {
			double A2 = calculateDifference(positions, A2PosA);
			A2 += calculateDifference(scaling, A2ScaleA);

			double B2 = calculateDifference(positions, B2Pos);
			B2 += calculateDifference(scaling, B2Scale);
			
			double A1 = calculateDifference(positions, A1Pos);
			A1 += calculateDifference(scaling, A1Scale);

			if(A2 < B2 && A2 < A1) {
				return 1;
			} else if (B2 < A1) {
				return 3;
			} else {
				return 0;
			}
		}

		return 10;
	}

	public static double calculateDifference(Vector2d[] found, Vector2d[] comparisons) {
		double dif = 0;
		for (int i = 0; i < found.length; i++) {
			dif += Math.abs(found[i].x - comparisons[i].x) + Math.abs(found[i].y - comparisons[i].y);
		}

		return dif;
	}
}
