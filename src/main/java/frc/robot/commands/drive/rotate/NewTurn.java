package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase; 
import frc.robot.RobotContainer;

public class NewTurn extends CommandBase {
    private Translation2d centerPoint;

    public NewTurn(Translation2d centerPoint) {
        addRequirements(RobotContainer.swerveDrive);
        this.centerPoint = centerPoint;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        Pose2d currentPose = RobotContainer.swerveDrive.getPose();

        // Top right
        double frontRightDiffY = Math.abs(currentPose.getY() - RobotContainer.swerveDrive.frontRightLocation.getY());
        double frontRightDistX = Math.abs(centerPoint.getX() - RobotContainer.swerveDrive.frontRightLocation.getX());

        double frontRightAngle = Math.atan(frontRightDistX / frontRightDiffY) - 90;
        /////////////////

        int furthestFromCenterIndex = 0;
        double furthestDist = Math.sqrt(frontRightDistX * frontRightDistX + frontRightDiffY * frontRightDiffY);

        // Top left
        double frontLeftDiffY = Math.abs(currentPose.getY() - RobotContainer.swerveDrive.frontLeftLocation.getY());
        double frontLeftDistX = Math.abs(centerPoint.getX() - RobotContainer.swerveDrive.frontLeftLocation.getX());

        double frontLeftAngle = Math.atan(frontLeftDistX / frontLeftDiffY) - 90;

        double frontLeftDist = Math.sqrt(frontLeftDistX * frontLeftDistX + frontLeftDiffY * frontLeftDiffY);

        if (frontLeftDist > furthestDist) {
            furthestFromCenterIndex = 1;
            furthestDist = frontLeftDist;
        }
        /////////////////

        // Bottom left
        double rearLeftDiffY = Math.abs(currentPose.getY() - RobotContainer.swerveDrive.rearLeftLocation.getY());
        double rearLeftDistX = Math.abs(centerPoint.getX() - RobotContainer.swerveDrive.rearLeftLocation.getX());

        double rearLeftAngle = Math.atan(rearLeftDistX / rearLeftDiffY) - 90;

        double rearLeftDist = Math.sqrt(rearLeftDistX * rearLeftDistX + rearLeftDiffY * rearLeftDiffY);

        if (rearLeftDist > furthestDist) {
            furthestFromCenterIndex = 2;
            furthestDist = rearLeftDist;
        }
        ///////////////

        // Bottom right
        double rearRightDiffY = Math.abs(currentPose.getY() - RobotContainer.swerveDrive.rearRightLocation.getY());
        double rearRightDistX = Math.abs(centerPoint.getX() - RobotContainer.swerveDrive.rearRightLocation.getX());

        double rearRightAngle = Math.atan(rearRightDistX / rearRightDiffY) - 90;

        double rearRightDist = Math.sqrt(rearRightDistX * rearRightDistX + rearRightDiffY * rearRightDiffY);

        if (rearRightDist > furthestDist) {
            furthestFromCenterIndex = 3;
            furthestDist = rearRightDist;
        }
        ///////////////
        
        Rotation2d frontLeftModuleRot = new Rotation2d(frontLeftAngle);
        SwerveModuleState frontLeftModule = new SwerveModuleState(0, frontLeftModuleRot);

        Rotation2d frontRightModuleRot = new Rotation2d(frontRightAngle);
        SwerveModuleState frontRightModule = new SwerveModuleState(0, frontRightModuleRot);

        Rotation2d rearLeftModuleRot = new Rotation2d(rearLeftAngle);
        SwerveModuleState rearLeftModule = new SwerveModuleState(0, rearLeftModuleRot);

        Rotation2d rearRightModuleRot = new Rotation2d(rearRightAngle);
        SwerveModuleState rearRightModule = new SwerveModuleState(0, rearRightModuleRot);

        RobotContainer.swerveDrive.setModuleStates(new SwerveModuleState[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule });
    }

    public Translation2d furthestFromCenter(Translation2d mod1, Translation2d mod2, Translation2d mod3, Translation2d mod4) {
        double mod1Distance = distanceFromCenter(mod1);
        double mod2Distance = distanceFromCenter(mod2);
        double mod3Distance = distanceFromCenter(mod3);
        double mod4Distance = distanceFromCenter(mod4);

        Translation2d largest = mod1;
        double largestDistance = mod1Distance;
        if (mod2Distance > largestDistance) {
            largest = mod2;
            largestDistance = mod2Distance;
        }
        if (mod3Distance > largestDistance) {
            largest = mod3;
            largestDistance = mod3Distance;
        }
        if (mod4Distance > largestDistance) {
            largest = mod4;
            largestDistance = mod4Distance;
        }

        return largest;
    }

    private double distanceFromCenter(Translation2d point) {
        double distance = Math
                .sqrt(Math.pow(point.getX() - centerPoint.getX(), 2) + Math.pow(point.getY() - centerPoint.getY(), 2));
        return distance;
    }
}