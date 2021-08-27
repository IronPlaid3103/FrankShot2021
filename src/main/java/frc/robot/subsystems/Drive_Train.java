// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.simulation.SimSparkMax;
import frc.robot.util.Settings;

public class Drive_Train extends SubsystemBase {
  private final CANSparkMax _frontLeftMotor = new SimSparkMax(Constants.DrivetrainConstants.frontLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _frontRightMotor = new SimSparkMax(Constants.DrivetrainConstants.frontRightMotor, MotorType.kBrushless);
  private final CANSparkMax _rearLeftMotor = new SimSparkMax(Constants.DrivetrainConstants.rearLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _rearRightMotor = new SimSparkMax(Constants.DrivetrainConstants.rearRightMotor, MotorType.kBrushless);


  private final MecanumDrive _drive = new MecanumDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor);
  private ADIS16470_IMU _gyro;

  private CANEncoder m_left_follower;
  private CANEncoder m_right_follower;

  private DifferentialDriveOdometry m_odometry;

  private Pose2d m_pose;

  private double _kP = Constants.LimelightConstants.kP;
  private double _kI = Constants.LimelightConstants.kI;
  private double _kD = Constants.LimelightConstants.kD;
  private double _kF = Constants.LimelightConstants.kF;

  private double _ksVolts = Constants.DrivetrainConstants.ksVolts;
  private double _kvVoltSecondsPerMeter = Constants.DrivetrainConstants.kvVoltSecondsPerMeter;
  private double _kaVoltSecondsSquaredPerMeter = Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter;

  private double _kachunkPower = Constants.DrivetrainConstants.kachunkPower;
  private double _kachunkTime = Constants.DrivetrainConstants.kachunkTime;
  private double _driveRightkP = Constants.DrivetrainConstants.driveRightkP;
  private double _nerf = 1.0;

  /** Creates a new Drive_Train. */
  public Drive_Train(ADIS16470_IMU gyro) {
    //Note: the following doesn't seem to work, so we had to add our own deadband function
    //_drive.setDeadband(Constants.DrivetrainConstants.deadband);

    _gyro = gyro;
    _gyro.setYawAxis(IMUAxis.kY);
    _gyro.reset();

    _frontLeftMotor.restoreFactoryDefaults();
    _frontRightMotor.restoreFactoryDefaults();
    _rearLeftMotor.restoreFactoryDefaults();
    _rearRightMotor.restoreFactoryDefaults();

    m_left_follower = _frontLeftMotor.getEncoder();
    m_right_follower = _frontRightMotor.getEncoder();

    m_left_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);
    m_right_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);

    encoderReset();
    
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(_gyro.getAngle()), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    enableOpenLoopRampRate(true);
  }

  public void enableOpenLoopRampRate(boolean enable) {
    double rampRate = (enable ? DrivetrainConstants.rampRate : 0.0);

    _frontLeftMotor.setOpenLoopRampRate(rampRate);
    _frontRightMotor.setOpenLoopRampRate(rampRate);
    _rearLeftMotor.setOpenLoopRampRate(rampRate);
    _rearRightMotor.setOpenLoopRampRate(rampRate);
  }
  
  public void teleopDrive(Joystick driver){
    double ySpeed = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_X));
    double xSpeed = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_Y));
    double zRotation = applyDeadband(driver.getRawAxis(Constants.JoystickConstants.RIGHT_STICK_X));

    double multiplier = .8; 
    _drive.driveCartesian(ySpeed * multiplier, -xSpeed * multiplier, zRotation * multiplier, _gyro.getAngle());
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < Constants.DrivetrainConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(Constants.DrivetrainConstants.deadband, value)) / (1 - Constants.DrivetrainConstants.deadband);
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    _drive.driveCartesian(ySpeed * _nerf, -xSpeed * _nerf, zRotation * _nerf);
  }

  public void encoderReset() {
    m_right_follower.setPosition(0.0);
    m_left_follower.setPosition(0.0);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_left_follower.getVelocity() / 60, -m_right_follower.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(_gyro.getAngle()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeftMotor.setVoltage(leftVolts);
    _frontRightMotor.setVoltage(-rightVolts);
    _rearRightMotor.setVoltage(-rightVolts);
    _rearLeftMotor.setVoltage(leftVolts);
  }

  public void setkP(double kP){
    _kP = kP;
  }

  public void setkI(double kI){
    _kI = kI;
  }

  public void setkD(double kD){
    _kD = kD;
  }

  public void setkF(double kF){
    _kF = kF;
  }

  public double getkP(){
    return _kP;
  }

  public double getkI(){
    return _kI;
  }

  public double getkD(){
    return _kD;
  }

  public double getkF(){
    return _kF;
  }

  public double getkaVoltSecondsSquaredPerMeter() {
    return _kaVoltSecondsSquaredPerMeter;
  }

  public double getkvVoltSecondsPerMeter() {
    return _kvVoltSecondsPerMeter;
  }

  public double getksVolts(){
    return _ksVolts;
  }

  public void setkaVoltSecondsSquaredPerMeter(double kaVoltSecondsSquaredPerMeter) {
    _kaVoltSecondsSquaredPerMeter = kaVoltSecondsSquaredPerMeter;
  }

  public void setkvVoltSecondsPerMeter(double kvVoltSecondsPerMeter) {
    _kvVoltSecondsPerMeter = kvVoltSecondsPerMeter;
  }

  public void setksVolts(double ksVolts){
    _ksVolts = ksVolts;
  }

  public void setKachunkPower(double power) {
    _kachunkPower = power;
  }

  public void setKachunkTime(double seconds) {
    _kachunkTime = seconds;
  }

  public double getKachunkPower() {
    return _kachunkPower;
  }

  public double getKachunkTime() {
    return _kachunkTime;
  }

  public void setDriveRightkP(double kP) {
    _driveRightkP = kP;
  }

  public double getDriveRightkP() {
    return _driveRightkP;
  }

  public void setNerf(double nerf) {
    if(nerf < 0) nerf = Math.abs(nerf);
    if(nerf > 1.0) nerf = 1.0;
    _nerf = nerf;
  }

  public double getNerf() {
    return _nerf;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pose = m_odometry.update(Rotation2d.fromDegrees(_gyro.getAngle()), m_left_follower.getPosition(), -m_right_follower.getPosition());
    
    setkP(Settings.getLiveDouble("Limelight", "kP", Constants.LimelightConstants.kP));
    setkI(Settings.getLiveDouble("Limelight", "kI", Constants.LimelightConstants.kI));
    setkD(Settings.getLiveDouble("Limelight", "kD", Constants.LimelightConstants.kD));
    setkF(Settings.getLiveDouble("Limelight", "kF", Constants.LimelightConstants.kF));
    
    setksVolts(Settings.getLiveDouble("DriveTrain", "ksVolts", Constants.DrivetrainConstants.ksVolts));
    setkvVoltSecondsPerMeter(Settings.getLiveDouble("DriveTrain", "kvVoltSecondsPerMeter", Constants.DrivetrainConstants.kvVoltSecondsPerMeter));
    setkaVoltSecondsSquaredPerMeter(Settings.getLiveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter));

    setKachunkPower(Settings.getLiveDouble("DriveTrain", "KachunkPower", Constants.DrivetrainConstants.kachunkPower));
    setKachunkTime(Settings.getLiveDouble("DriveTrain", "KachunkTime", Constants.DrivetrainConstants.kachunkTime));
    setDriveRightkP(Settings.getLiveDouble("DriveTrain", "DriveRightkP", Constants.DrivetrainConstants.driveRightkP));

    setNerf(Settings.getLiveDouble("DriveTrain", "Nerf", 1.0));

    SmartDashboard.putNumber("gyro", _gyro.getAngle());
  }
}
