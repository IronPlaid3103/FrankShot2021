// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.util.Settings;

public class Hopper extends SubsystemBase {
  private final CANSparkMax _hopperMotor = new CANSparkMax(HopperConstants.hopperMotor, MotorType.kBrushless);
  private final WPI_TalonFX _hopperFeederMotor = new WPI_TalonFX(HopperConstants.hopperFeederMotor);
  private double _hopperPower = Constants.HopperConstants.defaultPower;
  private double _hopperFeederPower = Constants.HopperConstants.defaultFeederPower;

  /** Creates a new Hopper. */
  public Hopper() {}

  public void stop() {
    _hopperMotor.stopMotor();
    // _hopperFeederMotor.stopMotor();
  }

  public void hopperGo() {
    _hopperMotor.set(_hopperPower);
    // _hopperFeederMotor.set(_hopperFeederPower);
  }

  public void hopperBack() {
    _hopperMotor.set(-_hopperPower);
    // _hopperFeederMotor.set(-_hopperFeederPower);
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

  public double getFeederPower(){
    return _hopperFeederPower;
  }

  @Override
  public void periodic() {
    _hopperPower = Settings.getLiveDouble("Hopper", "Power", Constants.HopperConstants.defaultPower);
    // _hopperFeederPower = Settings.getLiveDouble("Hopper", "FeederPower", Constants.HopperConstants.hopperFeederPower);
  }
}
