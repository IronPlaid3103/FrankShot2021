// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive_Train;
import frc.robot.util.Limelight;

public class LimelightAim extends CommandBase {
  private final Drive_Train _driveTrain;
  private final Limelight _limelight;
  private PIDController _PID;
  
  private int _aimCount = 0;

  public LimelightAim(Drive_Train driveTrain, Limelight limelight) {
    _driveTrain = driveTrain;
    _limelight = limelight;
    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _PID = new PIDController(_driveTrain.getkP(), _driveTrain.getkI(), _driveTrain.getkD());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!_limelight.isBypassed()) {
      double min_command = _driveTrain.getkF();
      double horizontalOffset = _limelight.getHorizontalOffset();
      double heading_error = _PID.calculate(horizontalOffset, 0);

      if(Math.abs(heading_error) < min_command) heading_error = Math.copySign(min_command, heading_error);
      _driveTrain.drive(0, 0, -heading_error);

      SmartDashboard.putNumber("Horizontal Offset", horizontalOffset);
      SmartDashboard.putNumber("Heading Error", heading_error);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("BYPASS", _limelight.isBypassed());
    if (_limelight.isBypassed()){
      return true;
    }
    double horizontalOffset = _limelight.getHorizontalOffset();
    if(Math.abs(horizontalOffset) < Constants.LimelightConstants.aimingTolerance && _limelight.isTargetValid())
      _aimCount++;
    else
      _aimCount = 0;

    SmartDashboard.putBoolean("isAimed", _aimCount >= 5);
    return (_aimCount >= 5);
  }
}
