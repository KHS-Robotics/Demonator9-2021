package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class TurnAroundPoint extends CommandBase {
    private Translation2d centerPoint;

    public TurnAroundPoint(Translation2d centerPoint) {
        addRequirements(RobotContainer.swerveDrive);
        this.centerPoint = centerPoint;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        RobotContainer.swerveDrive.resetPid();
    }

    public Translation2d furthestFromCenter(Translation2d mod1, Translation2d mod2, Translation2d mod3,
            Translation2d mod4) {
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

    @Override
    public void execute() {
        // Top right
        double frontRightDiffY = (centerPoint.getY() - RobotContainer.swerveDrive.frontRightLocation.getY());
        double frontRightDistX = (centerPoint.getX() - RobotContainer.swerveDrive.frontRightLocation.getX());

        double frontRightAngle = Math.atan2(frontRightDiffY, frontRightDistX) - Math.PI/2;

        double frontRightDist = Math.sqrt(frontRightDistX * frontRightDistX + frontRightDiffY * frontRightDiffY);
        /////////////////

        double furthestDist = frontRightDist;

        // Top left
        double frontLeftDiffY = (centerPoint.getY() - RobotContainer.swerveDrive.frontLeftLocation.getY());
        double frontLeftDistX = (centerPoint.getX() - RobotContainer.swerveDrive.frontLeftLocation.getX());

        double frontLeftAngle = Math.atan2(frontLeftDiffY, frontLeftDistX) - Math.PI/2;

        double frontLeftDist = Math.sqrt(frontLeftDistX * frontLeftDistX + frontLeftDiffY * frontLeftDiffY);

        if (frontLeftDist > furthestDist) {
            furthestDist = frontLeftDist;
        }
        /////////////////

        // Bottom left
        double rearLeftDiffY = (centerPoint.getY() - RobotContainer.swerveDrive.rearLeftLocation.getY());
        double rearLeftDistX = (centerPoint.getX() - RobotContainer.swerveDrive.rearLeftLocation.getX());

        double rearLeftAngle = Math.atan2(rearLeftDiffY, rearLeftDistX) - Math.PI/2;
        DriverStation.reportWarning("Rear left angle: " + rearLeftAngle, false);

        double rearLeftDist = Math.sqrt(rearLeftDistX * rearLeftDistX + rearLeftDiffY * rearLeftDiffY);

        if (rearLeftDist > furthestDist) {
            furthestDist = rearLeftDist;
        }
        ///////////////

        // Bottom right
        double rearRightDiffY = (centerPoint.getY() - RobotContainer.swerveDrive.rearRightLocation.getY());
        double rearRightDistX = (centerPoint.getX() - RobotContainer.swerveDrive.rearRightLocation.getX());

        double rearRightAngle = Math.atan2(rearRightDiffY, rearRightDistX) - Math.PI/2;

        double rearRightDist = Math.sqrt(rearRightDistX * rearRightDistX + rearRightDiffY * rearRightDiffY);

        if (rearRightDist > furthestDist) {
            furthestDist = rearRightDist;
        }
        ///////////////

        Rotation2d frontLeftModuleRot = new Rotation2d(frontLeftAngle);
        SwerveModuleState frontLeftModule = new SwerveModuleState(SwerveDrive.kMaxSpeed * (frontLeftDist / furthestDist), frontLeftModuleRot);

        Rotation2d frontRightModuleRot = new Rotation2d(frontRightAngle);
        SwerveModuleState frontRightModule = new SwerveModuleState(SwerveDrive.kMaxSpeed * (frontRightDist / furthestDist), frontRightModuleRot);

        Rotation2d rearLeftModuleRot = new Rotation2d(rearLeftAngle);
        SwerveModuleState rearLeftModule = new SwerveModuleState(SwerveDrive.kMaxSpeed * (rearLeftDist / furthestDist), rearLeftModuleRot);

        Rotation2d rearRightModuleRot = new Rotation2d(rearRightAngle);
        SwerveModuleState rearRightModule = new SwerveModuleState(SwerveDrive.kMaxSpeed * (rearRightDist / furthestDist), rearRightModuleRot);

        RobotContainer.swerveDrive.setModuleStates(
                new SwerveModuleState[] { frontLeftModule, frontRightModule, rearLeftModule, rearRightModule });

        RobotContainer.swerveDrive.updateOdometry();
    }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}