// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Settings;

public class Intake extends SubsystemBase {
  private final CANSparkMax _intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotor, MotorType.kBrushless);
  private double _power = Constants.IntakeConstants.defaultPower;
  
  /** Creates a new Intake. */
  public Intake() {}

  public void stop() {
    _intakeMotor.stopMotor();
  }

  public void intakeIn() {
    _intakeMotor.set(_power); 
  }

  public void setPower(double power){
    _power = power;
  }

  public double getPower(){
    return _power;
  }

  @Override
  public void periodic() {
    _power = Settings.getLiveDouble("Intake", "Power", Constants.IntakeConstants.defaultPower);
    // This method will be called once per scheduler run
  }
}
