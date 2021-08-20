// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterV2;
import frc.robot.subsystems.ShooterV2.COLOR;

public class ShooterV2Go extends CommandBase {
  private final ShooterV2 _shooter;
  private final Hopper _hopper;
  private final COLOR _color;
  private double _targetVelocity = 0.0;
  private int _velocityRampCounter = 0;

  /** Creates a new ShooterSimple. */
  public ShooterV2Go(ShooterV2 shooter, Hopper hopper, COLOR color) {
    _shooter = shooter;
    _hopper = hopper;
    addRequirements(_shooter, _hopper);

    _color = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _targetVelocity = _shooter.translateColorToVelocity(_color);
    _velocityRampCounter = 0;
    SmartDashboard.putBoolean("Shooter.AtV", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _shooter.shoot(_targetVelocity);

    SmartDashboard.putNumber("counter", _velocityRampCounter);
    System.out.println(_velocityRampCounter);

    if (_shooter.isAtTargetVelocity()) {
      if(++_velocityRampCounter > 15) {
        _hopper.hopperGo();
        SmartDashboard.putBoolean("Shooter.AtV", _shooter.isAtTargetVelocity());
      }
    }
    else
      _velocityRampCounter = 0;
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
