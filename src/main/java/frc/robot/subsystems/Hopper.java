// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  private final CANSparkMax _hopperMotor = new CANSparkMax(HopperConstants.hopperMotor, MotorType.kBrushless);

  /** Creates a new Hopper. */
  public Hopper() {}

  public void stop() {
    _hopperMotor.stopMotor();
  }

  public void hopperGo() {
    _hopperMotor.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
