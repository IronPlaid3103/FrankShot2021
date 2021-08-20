// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Settings;

public class ShooterV2 extends SubsystemBase {
  public enum COLOR{Red, Yellow, Blue, Green}

  WPI_TalonFX _shooterMotor1 = new WPI_TalonFX(ShooterConstants.shooterMotor1);
  WPI_TalonFX _shooterMotor2 = new WPI_TalonFX(ShooterConstants.shooterMotor2);

  double _targetVelocity_UnitsPer100ms;
  
  /** Creates a new ShooterSimple. */
  public ShooterV2() {
    setDefaultCommand(new RunCommand(this::stop, this));

    _shooterMotor1.configFactoryDefault();
    _shooterMotor1.setNeutralMode(NeutralMode.Coast);
		
		/* Config neutral deadband to be the smallest possible */
		_shooterMotor1.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    _shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);
											
		/* Config the peak and nominal outputs */
		_shooterMotor1.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		_shooterMotor1.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		_shooterMotor1.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
		_shooterMotor1.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		_shooterMotor1.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains_Velocit.kF, ShooterConstants.kTimeoutMs);
		_shooterMotor1.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains_Velocit.kP, ShooterConstants.kTimeoutMs);
		_shooterMotor1.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains_Velocit.kI, ShooterConstants.kTimeoutMs);
    _shooterMotor1.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains_Velocit.kD, ShooterConstants.kTimeoutMs);
    
    _shooterMotor1.setInverted(TalonFXInvertType.Clockwise);

    _shooterMotor2.follow(_shooterMotor1);
    _shooterMotor2.setInverted(TalonFXInvertType.CounterClockwise);
  }

  public void stop() {
    _targetVelocity_UnitsPer100ms = 0;
    _shooterMotor1.set(0);
  }

  public void shoot(double targetVelocity) {
    /**
     * 2048 Units/Rev * RPM / 600 100ms/min in either direction:
     * velocity setpoint is in units/100ms
     */
    _targetVelocity_UnitsPer100ms = targetVelocity * 2048.0 / 600.0;
    _shooterMotor1.set(TalonFXControlMode.Velocity, _targetVelocity_UnitsPer100ms);

    SmartDashboard.putNumber("Shooter.T", targetVelocity);
  }

  public boolean isAtTargetVelocity() {
    if (Math.abs(_shooterMotor1.getClosedLoopError()) < _targetVelocity_UnitsPer100ms * 0.05)
      return true;
    else
      return false;
  }

  public double translateColorToVelocity(COLOR color){
    switch(color){
      case Red: 
        return getRedVelocity();
      case Blue: 
        return getBlueVelocity();
      case Yellow: 
        return getYellowVelocity();
      case Green:
        return getGreenVelocity();
      default:
        return 0.0;
    }
  }

  public void setRedVelocity(double redVelocity){
    ShooterConstants.redVelocity = redVelocity;
  }

  public void setBlueVelocity(double blueVelocity){
    ShooterConstants.blueVelocity = blueVelocity;
  }

  public void setYellowVelocity(double yellowVelocity){
    ShooterConstants.yellowVelocity = yellowVelocity;
  }

  public void setGreenVelocity(double greenVelocity){
    ShooterConstants.greenVelocity = greenVelocity;
  }

  public double getRedVelocity(){
    return ShooterConstants.redVelocity;
  }

  public double getBlueVelocity(){
    return ShooterConstants.blueVelocity;
  }

  public double getYellowVelocity(){
    return ShooterConstants.yellowVelocity;
  }

  public double getGreenVelocity(){
    return ShooterConstants.greenVelocity;
  }

  @Override
  public void periodic() {
    setRedVelocity(Settings.getLiveDouble("Shooter", "RedVelocity", ShooterConstants.redVelocity));
    setGreenVelocity(Settings.getLiveDouble("Shooter", "GreenVelocity", ShooterConstants.greenVelocity));
    setBlueVelocity(Settings.getLiveDouble("Shooter", "BlueVelocity", ShooterConstants.blueVelocity));
    setYellowVelocity(Settings.getLiveDouble("Shooter", "YellowVelocity", ShooterConstants.yellowVelocity));
    
    SmartDashboard.putNumber("Shooter.V", _shooterMotor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter.RPM", (_shooterMotor1.getSelectedSensorVelocity() / 2048.0 * 600.0));
  }
}
