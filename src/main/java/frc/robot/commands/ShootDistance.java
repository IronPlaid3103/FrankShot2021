// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.COLOR;
import frc.robot.util.Limelight;

public class ShootDistance extends CommandBase {
  /** Creates a new ShootDistance. */
  private final Limelight _limelight;
  private final Shooter _shooter;
  private final Hopper _hopper;
  private final COLOR _color;

  public ShootDistance(Limelight limelight, Shooter shooter, Hopper hopper, COLOR color) {
    // Use addRequirements() here to declare subsystem dependencies.
    _limelight = limelight;
    _shooter = shooter;
    _hopper = hopper;
    _color = color;
    addRequirements(_shooter, _hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!_limelight.isBypassed()){
      double distance = _limelight.getDistance();
      if (distance <= 90){
        _shooter.setColor(COLOR.Green);
        SmartDashboard.putString("ShootingZone", "Green");
      }
      else if (distance > 90 && distance <=  150){
        _shooter.setColor(COLOR.Yellow);
        SmartDashboard.putString("ShootingZone", "Yellow");
      }
      else if (distance > 150 && distance <= 210){
        _shooter.setColor(COLOR.Blue);
        SmartDashboard.putString("ShootingZone", "Blue");
      }
      else {
        _shooter.setColor(COLOR.Red);
        SmartDashboard.putString("ShootingZone", "Red");
      }
      SmartDashboard.putNumber("Limelight Distance", distance);
    }
    else {
      _shooter.setColor(_color);
    }
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
