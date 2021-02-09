// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive_Train extends SubsystemBase {
  private final CANSparkMax _flDrive = new CANSparkMax(Constants.DrivetrainConstants.flDrive, MotorType.kBrushless);
  private final CANSparkMax _frDrive = new CANSparkMax(Constants.DrivetrainConstants.frDrive, MotorType.kBrushless);
  private final CANSparkMax _blDrive = new CANSparkMax(Constants.DrivetrainConstants.blDrive, MotorType.kBrushless);
  private final CANSparkMax _brDrive = new CANSparkMax(Constants.DrivetrainConstants.brDrive, MotorType.kBrushless);

  private final ADIS16470_IMU _gyro = new ADIS16470_IMU();

  private final MecanumDrive _drive = new MecanumDrive(_flDrive, _brDrive, _frDrive, _brDrive);

  //TODO: We want to use the encoders from the NEO/SparkMax - the motor controller objects above
  //      have a method to get an encoder (.getEncoder()), but it returns a CANEncoder, so we need
  //      to determine how to use that (the methods that replace it in all the code below)
  public Encoder m_left_follower = new Encoder(1, 2);
  public Encoder m_right_follower = new Encoder(3, 4);

  DifferentialDriveOdometry m_odometry;

  /** Creates a new Drive_Train. */
  public Drive_Train() {
    //Note: the following doesn't seem to work, so we had to add our own deadband function
    //_drive.setDeadband(Constants.DrivetrainConstants.deadband);

    _gyro.reset();

    m_left_follower.setDistancePerPulse(40); // dont know what distance per pulse is
    m_right_follower.setDistancePerPulse(40);

    encoderReset();
    
    m_odometry = new DifferentialDriveOdometry(_gyro.getRotation2d());
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
    m_right_follower.reset();
    m_left_follower.reset();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_left_follower.getRate(), m_right_follower.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pose, _gyro.getRotation2d());
  }

  public void arcadeDrive(double xDirection, double yDirection, double rotation) {
    _drive.driveCartesian(yDirection, xDirection, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _flDrive.setVoltage(leftVolts);
    _frDrive.setVoltage(rightVolts);
    _brDrive.setVoltage(rightVolts);
    _blDrive.setVoltage(leftVolts);
  }

  public double getAverageEncoderDistance() {
    return ((m_left_follower.getDistance() + m_right_follower.getDistance()) /2);
  }

  public Encoder getLeftEncoder() {
    return m_left_follower;
  }

  public Encoder getRightEncoder() {
    return m_right_follower;
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
    m_odometry.update(_gyro.getRotation2d(), m_left_follower.getDistance(), m_right_follower.getDistance());
  }
}
