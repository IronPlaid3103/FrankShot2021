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

  public ShootDistance(Limelight limelight, Shooter shooter, Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    _limelight = limelight;
    _shooter = shooter;
    _hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = _limelight.getDistance();
    if (distance <= 90){
      _shooter.setColor(COLOR.Green);
      SmartDashboard.putString("Color", "Green");
    }
    else if (distance > 90 && distance <=  150){
      _shooter.setColor(COLOR.Yellow);
      SmartDashboard.putString("Color", "Yellow");
    }
    else if (distance > 150 && distance <= 210){
      _shooter.setColor(COLOR.Blue);
      SmartDashboard.putString("Color", "Blue");
    }
    else {
      _shooter.setColor(COLOR.Red);
      SmartDashboard.putString("Color", "Red");
    }
    _shooter.shoot();
    _hopper.hopperGo();
    SmartDashboard.putNumber("Distance", distance);
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
