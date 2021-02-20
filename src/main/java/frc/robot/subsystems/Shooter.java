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

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX _shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor);
  private double _targetRPM;

  //TODO: move PIDF values to constants for a default, dashboard for user updates, and persist on the robot using Preferences (PIDF values 2020 code - RobotContainer)
  private double _kF = 10;
  private double _kP = 10;

  /** Creates a new Shooter. */
  public Shooter() {
    setDefaultCommand(new RunCommand(this::stop, this));
  }

  public void setTargetRPM (double rpm) {
    _targetRPM = rpm;
  }

  public void setKf(double kF) {
    _kF = kF;
  }

  public void setKp(double kP) {
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
