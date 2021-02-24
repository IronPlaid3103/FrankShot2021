// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Settings;

public class Shooter extends SubsystemBase {
  public enum COLOR{Red, Yellow, Blue, Green}
  private final WPI_TalonFX _shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor);
  private double _targetRPM;
  private COLOR _color;

  //TODO: move PIDF values to constants for a default, dashboard for user updates, and persist on the robot using Preferences (PIDF values 2020 code - RobotContainer)
  private double _kF = Constants.ShooterConstants.defaultkF;
  private double _kP = Constants.ShooterConstants.defaultkP;

  /** Creates a new Shooter. */
  public Shooter() {
    setDefaultCommand(new RunCommand(this::stop, this));
  }

  public void setTargetRPM (double rpm) {
    _targetRPM = rpm;
  }

  public void setkF(double kF) {
    _kF = kF;
  }

  public void setkP(double kP) {
    _kP = kP;
  }

  public double getTargetRPM() {
    return _targetRPM;
  }

  public double getkF() {
    return _kF;
  }

  public double getkP() {
    return _kP;
  }

  public boolean isAtTargetRPM() {
    return (Math.abs(_shooterMotor.getClosedLoopError()) <= _targetRPM * .05);
  }

  public void shoot() {
    _shooterMotor.set(ControlMode.Velocity, _targetRPM, DemandType.ArbitraryFeedForward, _kF);
  }

  public void stop() {
    _shooterMotor.set(0);
  }

  public void setRedVelocity(double redVelocity){
    Constants.ShooterConstants.redVelocity = redVelocity;
  }

  public void setBlueVelocity(double blueVelocity){
    Constants.ShooterConstants.blueVelocity = blueVelocity;
  }

  public void setYellowVelocity(double yellowVelocity){
    Constants.ShooterConstants.yellowVelocity = yellowVelocity;
  }

  public void setGreenVelocity(double greenVelocity){
    Constants.ShooterConstants.greenVelocity = greenVelocity;
  }

  public double getRedVelocity(){
    return Constants.ShooterConstants.redVelocity;
  }

  public double getBlueVelocity(){
    return Constants.ShooterConstants.blueVelocity;
  }

  public double getYellowVelocity(){
    return Constants.ShooterConstants.yellowVelocity;
  }

  public double getGreenVelocity(){
    return Constants.ShooterConstants.greenVelocity;
  }

  public void setColor(COLOR color){
    _color = color;
  }


  @Override
  public void periodic() {
    _kF = Settings.getLiveDouble("Shooter", "kF", Constants.ShooterConstants.defaultkF);
    _kP = Settings.getLiveDouble("Shooter", "kP", Constants.ShooterConstants.defaultkP);
    setRedVelocity(Settings.getLiveDouble("Shooter", "RedVelocity", Constants.ShooterConstants.redVelocity));
    setGreenVelocity(Settings.getLiveDouble("Shooter", "GreenVelocity", Constants.ShooterConstants.greenVelocity));
    setBlueVelocity(Settings.getLiveDouble("Shooter", "BlueVelocity", Constants.ShooterConstants.blueVelocity));
    setYellowVelocity(Settings.getLiveDouble("Shooter", "YellowVelocity", Constants.ShooterConstants.yellowVelocity));
  }
}
