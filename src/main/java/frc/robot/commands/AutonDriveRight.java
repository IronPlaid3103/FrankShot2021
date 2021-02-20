// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Train;
import frc.robot.util.LIDARLiteV3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class AutonDriveRight extends CommandBase {
  private final Drive_Train _drivetrain;
  private Timer _timer = new Timer();
  private final double kP = 1;
  private ADIS16470_IMU _gyro;
  private LIDARLiteV3 _lidar;

  /** Creates a new DriveRight. */
  public AutonDriveRight(Drive_Train drivetrain, ADIS16470_IMU gyro, LIDARLiteV3 lidar) {
    _drivetrain = drivetrain;
    _gyro = gyro;
    _lidar = lidar;
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
    double error = -_gyro.getRate();
    _drivetrain.drive(0.2, 0, 0 -kP * error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.drive(0, 0, 0);
    _timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distance = _lidar.getDistanceInches(true);
    
    String path = "";

    if (distance >= 144 && distance <= 156) {
      path = "ARed";
    } else if (distance >= 174 && distance <= 186) {
      path = "ABlue";
    } else if (distance >= 54 && distance <= 66) {
      path = "BRed";
    } else if (distance >= 204 && distance <= 216) {
      path = "BBlue";
    }
    if (path.length() > 0) {
      NetworkTableInstance.getDefault().getTable("Frank").getEntry("GalacticSearchPath").setString(path);
    }
    return path.length() > 0;
  }
}
