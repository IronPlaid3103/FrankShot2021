// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.simulation.SimSparkMax;

public class Drive_Train extends SubsystemBase {
  private final CANSparkMax _frontLeftMotor = new SimSparkMax(Constants.DrivetrainConstants.frontLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _frontRightMotor = new SimSparkMax(Constants.DrivetrainConstants.frontRightMotor, MotorType.kBrushless);
  private final CANSparkMax _rearLeftMotor = new SimSparkMax(Constants.DrivetrainConstants.rearLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _rearRightMotor = new SimSparkMax(Constants.DrivetrainConstants.rearRightMotor, MotorType.kBrushless);


  private final MecanumDrive _drive = new MecanumDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
  private ADIS16470_IMU _gyro;

  private CANEncoder m_left_follower;
  private CANEncoder m_right_follower;

  DifferentialDriveOdometry m_odometry;

  /** Creates a new Drive_Train. */
  public Drive_Train(ADIS16470_IMU gyro) {
    //Note: the following doesn't seem to work, so we had to add our own deadband function
    //_drive.setDeadband(Constants.DrivetrainConstants.deadband);

    _gyro = gyro;
    _gyro.reset();

    m_left_follower = _frontLeftMotor.getEncoder();
    m_right_follower = _frontRightMotor.getEncoder();

    m_left_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters * DrivetrainConstants.kGearReduction);
    m_right_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters * DrivetrainConstants.kGearReduction);

    encoderReset();
    
    m_odometry = new DifferentialDriveOdometry(_gyro.getRotation2d());

    _frontLeftMotor.setOpenLoopRampRate(DrivetrainConstants.rampRate);
    _frontRightMotor.setOpenLoopRampRate(DrivetrainConstants.rampRate);
    _rearLeftMotor.setOpenLoopRampRate(DrivetrainConstants.rampRate);
    _rearRightMotor.setOpenLoopRampRate(DrivetrainConstants.rampRate);
  }
  
  public void teleopDrive(Joystick driver){
    double ySpeed = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_X));
    double xSpeed = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_Y));
    double zRotation = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.RIGHT_STICK_X));

    _drive.driveCartesian(ySpeed, -xSpeed, zRotation, _gyro.getAngle());
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < Constants.DrivetrainConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(Constants.DrivetrainConstants.deadband, value)) / (1 - Constants.DrivetrainConstants.deadband);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    _drive.driveCartesian(ySpeed, -xSpeed, zRotation);
  }

  public void encoderReset() {
    m_right_follower.setPosition(0.0);
    m_left_follower.setPosition(0.0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_left_follower.getVelocity(), m_right_follower.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pose, _gyro.getRotation2d());
  }

  public void arcadeDrive(double xDirection, double yDirection, double rotation) {
    _drive.driveCartesian(yDirection, xDirection, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeftMotor.setVoltage(leftVolts);
    _frontRightMotor.setVoltage(rightVolts);
    _rearRightMotor.setVoltage(rightVolts);
    _rearLeftMotor.setVoltage(leftVolts);
  }

  public double getAverageEncoderDistance() {
    return ((m_left_follower.getPosition() + m_right_follower.getPosition()) /2);
  }


  public void setMaxOutput(double maxOutput) {
    _drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    _gyro.reset();
  }

  public double getHeading() {
    return _gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -_gyro.getRate();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(_gyro.getRotation2d(), m_left_follower.getPosition(), m_right_follower.getPosition());
    
    double angle = _gyro.getAngle();
    SmartDashboard.putNumber("gyro", Math.floor(angle * 100)/100);
  }
}
