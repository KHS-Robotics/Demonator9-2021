/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import javax.swing.plaf.RootPaneUI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.rotate.TurnAroundPointArc;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.drive.rotate.RotateToTarget;
// import frc.robot.commands.hood.AlignHoodToTarget;
// import frc.robot.commands.indexer.SetIndexerAuto;
// import frc.robot.commands.shooter.RampShooter;
// import frc.robot.commands.shooter.ShootAuto;
// import frc.robot.vision.Limelight;

/**
 * Class to generate commands for autonomous movement
 */
public class AutoCommands {
  // private static boolean initialized = false;
  //public static SwerveControllerCommand wallLineUp, frontTrench, pickTrench, returnTrench, moveOffInit, steal, moveFromSteal, pick3Rendevous, shootFromRendevous;
  public static SwerveControllerCommand test, redAStart, redAOne, redATwo, redAEnd, bounce, barrel, slalom, barrelA, testB;
  public static Trajectory barrel1;
  public static TrajectoryConfig config = new TrajectoryConfig(3.0, 6.0);

  public static void autoInit() {
    // if(!initialized) {
      new Thread(() -> {
        //redAStart = loadPathweaverTrajectory("RedAStart");
        /*redAOne = loadPathweaverTrajectory("RedAC3toD5");
        redATwo = loadPathweaverTrajectory("RedAD5toA6");
        redAEnd = loadPathweaverTrajectory("RedAEnd");*/

        bounce = loadPathweaverTrajectory("Bounce");
        barrel = loadPathweaverTrajectory("Barrel");
        slalom = loadPathweaverTrajectory("Slalom");

        test = loadPathweaverTrajectory("Test");
        
        config.setKinematics(RobotContainer.swerveDrive.kinematics);

        var start = new Pose2d(1.288, 2.286, Rotation2d.fromDegrees(0));
        var end = new Pose2d(4, 2.348, Rotation2d.fromDegrees(0));

        var interiorPoints = new ArrayList<Translation2d>();
        interiorPoints.add(new Translation2d(2.626, 2.436));

        barrel1 = TrajectoryGenerator.generateTrajectory(start, interiorPoints, end, config);
        barrelA = getCommandFromTrajectory(barrel1);

        //testB = getCommandFromTrajectory(testTrajectory2);

        redAStart = generatePath(new Pose2d(9, 2.286, Rotation2d.fromDegrees(0)), new Translation2d[] {}, new Pose2d(7.714, 2.266, Rotation2d.fromDegrees(0)));
        redAOne = generatePath(new Pose2d(7.714, 2.66, Rotation2d.fromDegrees(0)), new Translation2d[] {}, new Pose2d(6.248, 1.544, Rotation2d.fromDegrees(-15)));
        redATwo = generatePath(new Pose2d(6.248, 1.544, Rotation2d.fromDegrees(-15)), new Translation2d[] {}, new Pose2d(5.414, 3.833, Rotation2d.fromDegrees(75)));
        redAEnd = generatePath(new Pose2d(5.414, 3.833, Rotation2d.fromDegrees(75)), new Translation2d[] {}, new Pose2d(.5, 2.286, Rotation2d.fromDegrees(0)));
      }).start();

      //initialized = true;
   // }
  }
    
  public static SwerveControllerCommand generatePath(Pose2d start, Translation2d[] mid, Pose2d end) {
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    for(int i = 0; i < mid.length; i++) {
      interiorWaypoints.add(mid[i]);
    }

    return getCommandFromTrajectory(TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config));
  }


  public static SwerveControllerCommand loadPathweaverTrajectory(String json) {
      Trajectory trajectory;
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + json + ".wpilib.json");
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
        return null;
      }

      return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.8, 0.001, 0.5), // x (forward/backwards)
        new PIDController(0.8, 0.001, 0.5), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive
      );
  }

  public static SwerveControllerCommand getCommandFromTrajectory(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive
      );
  }

  public static Command groupARed() {
    return redAStart.andThen(redAOne).andThen(redATwo).andThen(redAEnd);
  }

  public static Command groupABlue() {
  //   return
  //     groupAStart
  //     .andThen(groupABlue)
  //     .andThen(endGroupABlue);
    return null;
  }

  public static Command groupBRed() {
      return
        test;
        // .andThen(groupBRed)
        // .andThen(endGroupBRed);
  }

  public static Command groupBBlue() {
  //     return
  //       groupBStart
  //       .andThen(groupBBlue)
  //       .andThen(endGroupBBlue);
    return null;
  }

  public static Command barrelRun() {
    //return getCommandFromTrajectory(testTrajectory);
    return barrel;
  }

  public static Command bounce() {
    return bounce;
  }

  public static Command slalom() {
    return slalom;
  }

  public static Command barrelRunNew() {
    return barrelA.andThen(new TurnAroundPointArc(new Translation2d(1,1), true, 360));
  }
}