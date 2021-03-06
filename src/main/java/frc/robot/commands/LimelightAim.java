// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive_Train;
import frc.robot.util.Limelight;

public class LimelightAim extends CommandBase {
  private final Drive_Train _driveTrain;
  private final Limelight _limelight;
  /** Creates a new LimelightAim. */

  public LimelightAim(Drive_Train driveTrain, Limelight limelight) {
    _driveTrain = driveTrain;
    _limelight = limelight;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = -0.1;
    double min_command = 0.05;

    double horizontalOffset = _limelight.getHorizontalOffset();
    double heading_error = -horizontalOffset;
    double steering_rotate = 0.0;

    if (horizontalOffset > 1.0) {
      steering_rotate = kP * heading_error - min_command;
    } else if (horizontalOffset < 1.0) {
      steering_rotate = kP * heading_error + min_command;
    }

    _driveTrain.drive(0, 0, steering_rotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double horizontalOffset = _limelight.getHorizontalOffset();
    boolean isAimed = Math.abs(horizontalOffset) < Constants.LimelightConstants.aimingTolerance && _limelight.isTargetValid();
    SmartDashboard.putBoolean("isAimed", isAimed);
    if (isAimed){
      return true;
    }
    return false;
  }
}
