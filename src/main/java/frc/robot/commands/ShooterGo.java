// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ShooterGo extends CommandBase {
  private final Shooter _shooter;
  private final Hopper _hopper;
  private double _targetRPM;

  /** Creates a new ShooterGo. */
  public ShooterGo(Shooter shooter, Hopper hopper, double targetRPM) {
    _shooter = shooter;
    addRequirements(_shooter);
    _hopper = hopper;
    addRequirements(_hopper);
    _targetRPM = targetRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.setTargetRPM(_targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.shoot();
    if (_shooter.isAtTargetRPM()){
      _hopper.hopperGo();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
