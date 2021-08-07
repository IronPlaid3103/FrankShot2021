// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.Settings;

public class Hopper extends SubsystemBase {
  private final CANSparkMax _hopperMotor = new CANSparkMax(HopperConstants.hopperMotor, MotorType.kBrushless);
  // private final WPI_TalonSRX _hopperMotor = new WPI_TalonSRX(HopperConstants.hopperMotor);
  private final CANSparkMax _hopperFeederMotor = new CANSparkMax(HopperConstants.hopperFeederMotor, MotorType.kBrushless);
  private double _hopperPower = Constants.HopperConstants.defaultPower;
  private double _hopperFeederPower = Constants.HopperConstants.defaultFeederPower;
  private double _hopperFeederIdle = Constants.HopperConstants.defaultFeederIdle;  

  /** Creates a new Hopper. */
  public Hopper() { }

  public void stop() {
    _hopperMotor.stopMotor();
    _hopperFeederMotor.stopMotor();
  }

  public void idle() {
    _hopperMotor.set(_hopperPower);
    _hopperFeederMotor.set(_hopperFeederIdle);
  }

  public void hopperGo() {
    _hopperMotor.set(_hopperPower);
    _hopperFeederMotor.set(_hopperFeederPower);
  }

  public void hopperBack() {
    _hopperMotor.set(-_hopperPower);
    _hopperFeederMotor.set(-_hopperFeederIdle);
  }

  public void setPower(double power){
    _hopperPower = power;
  }

  public double getPower(){
    return _hopperPower;
  }

  public void setFeederPower(double power){
    _hopperFeederPower = power;
  }

  public void setFeederIdle(double power){
    _hopperFeederIdle = power;
  }

  public double getFeederPower(){
    return _hopperFeederPower;
  }

  public double getFeederIdle(){
    return _hopperFeederIdle;
  }

  @Override
  public void periodic() {
    _hopperPower = Settings.getLiveDouble("Hopper", "Power", Constants.HopperConstants.defaultPower);
    _hopperFeederPower = Settings.getLiveDouble("Hopper", "FeederPower", Constants.HopperConstants.defaultFeederPower);
    _hopperFeederIdle = Settings.getLiveDouble("Hopper", "FeederIdle", Constants.HopperConstants.defaultFeederIdle);
  }
}
