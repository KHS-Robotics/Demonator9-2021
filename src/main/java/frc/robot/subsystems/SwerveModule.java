/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Swerve Module
 */
public class SwerveModule extends SubsystemBase {
  public final String name;

  private boolean isFlipped;
  private final boolean isInverted;

  private final CANSparkMax driveMotor;
  private final CANEncoder driveEncoder;

  private final CANSparkMax pivotMotor;
  private final CANEncoder pivotEncoder;
  private final CANPIDController drivePID;

  private final PIDController pivotPID;
  private final DigitalInput setDetection;

  /**
   * Constructs a Swerve Module.
   * @param name the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP P value of Pivot PID
   * @param pivotI I value of Pivot PID
   * @param pivotD D value of Pivot PID
   * @param digitalInputPort port number for the digital input, used to calibrate pivots
   * @param reversed true if drive motor is reversed
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI, double pivotD, double driveP, double driveI, double driveD, double driveFF, int digitalInputPort, boolean reversed) {
    isInverted = reversed;

    this.name = name;

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(pivotMotorChannel, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(30);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);

    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPositionConversionFactor(360.0 / 18.0); // 360 degree per rotation, 18:1 -> 360 * 1/18

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setVelocityConversionFactor(Constants.DRIVE_VEL_ENCODER); // 4" diameter wheel (0.0508 meter radius), 8.33:1 -> 2*pi*0.0508 / 8.33
    driveEncoder.setPositionConversionFactor(Constants.DRIVE_POS_ENCODER); // 4" diameter wheel (0.0508 meter radius), 8.33:1 -> 2*pi*0.0508 / 8.33

    drivePID = driveMotor.getPIDController();
    drivePID.setP(driveP);
    drivePID.setI(driveI);
    drivePID.setD(driveD);
    drivePID.setFF(driveFF);
    drivePID.setIZone(1);

    pivotPID = new PIDController(pivotP, pivotI, pivotD);
    pivotPID.enableContinuousInput(-180, 180);
    pivotPID.setTolerance(1);

    setDetection = new DigitalInput(digitalInputPort);

    var tab = Shuffleboard.getTab(name + " Module");
    //tab.addNumber("Angle (Deg)", this::getAngle);
    tab.addNumber("Setpoint (Deg)", pivotPID::getSetpoint);
    tab.addNumber("Error (Deg)", pivotPID::getPositionError);
    //tab.addNumber("Speed mps", driveEncoder::getVelocity);
    tab.addBoolean("atSetpoint", pivotPID::atSetpoint);
    tab.addBoolean("isFlipped", () -> isFlipped);
    tab.addBoolean("isCenter", () -> !this.setDetection.get());
  }

  /**
   * Constructs a Swerve Module.
   * @param name the name/position of the module
   * @param driveMotorChannel CAN ID for the drive motor
   * @param pivotMotorChannel CAN ID for the pivot motor
   * @param pivotP P value of Pivot PID
   * @param pivotI I value of Pivot PID
   * @param pivotD D value of Pivot PID
   * @param digitalInputPort port number for the digital input, used to calibrate pivots
   */
  public SwerveModule(String name, int driveMotorChannel, int pivotMotorChannel, double pivotP, double pivotI, double pivotD, double driveP, double driveI, double driveD, double driveFF, int digitalInputPort) {
    this(name, driveMotorChannel, pivotMotorChannel, pivotP, pivotI, pivotD, driveP, driveI, driveD, driveFF, digitalInputPort, false);
  }

  @Override
  public void periodic() {
  }

  /**
   * Returns the current state of the module.
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState((isInverted ? -1 : 1) * driveEncoder.getVelocity(), Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Sets the PID values for the pivot module.
   * @param p the p value for the pivot module
   * @param i the i value for the pivot module
   * @param d the d value for the pivot module
   */
  public void setPid(double p, double i, double d) {
    pivotPID.setPID(p, i, d);
  }

  /**
   * Sets the desired state for the module.
   * @param state desired state with the speed and angle
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(SwerveModuleState state, boolean useShortestPath) {
    pivotMotor.set(MathUtil.clamp(pivotPID.calculate(getAngle(), useShortestPath ? calculateShortestPath(state.angle.getDegrees()) : state.angle.getDegrees()), -1, 1));
    drivePID.setReference(state.speedMetersPerSecond*(isInverted ? -1 : 1)*(isFlipped && useShortestPath ? -1 : 1), ControlType.kVelocity); 
  }

  /**
   * Sets the desired state for the module.
   * @param state desired state with the speed and angle
   */
  public void setDesiredState(SwerveModuleState state) {
    setDesiredState(state, true);
  }

  /**
   * Sets the desired state for the module.
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   * @param useShortestPath whether or not to use the shortest path
   */
  public void setDesiredState(double speed, double angle, boolean useShortestPath) {
    setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(angle)), useShortestPath);
  }

  /**
   * Sets the desired state for the module.
   * @param speed the desired speed in meters/second
   * @param angle the desired angle in degrees from [-180, 180]
   */
  public void setDesiredState(double speed, double angle) {
    setDesiredState(speed, angle, true);
  }

  /**
   * Gets the angle of the pivot module.
   * @return the angle of the pivot module ranging from [-180,180]
   */
  public double getAngle() {
    var angle = pivotEncoder.getPosition();
    if(angle > 0) {
      angle %= 360;
      if(angle > 180) {
        angle -= 360;
      }
    }
    else if(angle < 0) {
      angle %= -360;
      if(angle < -180) {
        angle += 360;
      }
    }
    return angle;
  }

  /**
   * Stops the module.
   */
  public void stop() {
    driveMotor.set(0);
    pivotMotor.set(0);
    pivotPID.reset();
  }

  /**
   * For homing the modules
   * @return true if module is homed
   */
  public boolean resetEncoder() {
    if(setDetection.get()) { // sensor inverted
      pivotMotor.set(0.04);
      return false;
    } else {
      pivotMotor.set(0);
      pivotEncoder.setPosition(0.0);
      return true;
    }
  }

  /**
   * Calculates the shortest path the pivot module should take, it
   * might be the given <code>targetAngle</code>. Flips the drive motor
   * if there is a shorter path.
   * @param targetAngle the desired angle of the module
   * @return the shortest path to the target angle, flips the
   * drive motor if there is a shorter path
   */
  private double calculateShortestPath(double targetAngle) {
    var currentAngle = this.getAngle();
    var dAngle = Math.abs(targetAngle - currentAngle);
    
    isFlipped = dAngle > 90 && dAngle < 270;
    
    if(isFlipped) {
      if(targetAngle > 0 || targetAngle == 0 && currentAngle < 0) {
        targetAngle -= 180;
      }
      else if(targetAngle < 0 || targetAngle == 0 && currentAngle > 0) {
        targetAngle += 180;
      }
    }
    
    return targetAngle;
  }
}
