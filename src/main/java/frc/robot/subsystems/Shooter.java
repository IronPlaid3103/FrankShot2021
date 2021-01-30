// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX _shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.shooterMotor);

  /** Creates a new Shooter. */
  public Shooter() {}

  public void stop() {
    _shooterMotor.stopMotor();
  }

  public void shooterGo() {
    _shooterMotor.set(ControlMode.Velocity, Constants.ShooterConstants.defaultVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
