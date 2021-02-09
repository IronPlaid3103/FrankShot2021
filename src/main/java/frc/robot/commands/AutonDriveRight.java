// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;
import edu.wpi.first.wpilibj.Timer;

public class AutonDriveRight extends CommandBase {
  private final Drive_Train _drivetrain;
  private Timer _timer = new Timer();

  /** Creates a new DriveRight. */
  public AutonDriveRight(Drive_Train drivetrain) {
    _drivetrain = drivetrain;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.drive(0.5, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.drive(0,0,0);
    _timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_timer.get() >= 5)
      return true;
    else
      return false;
  }
}
