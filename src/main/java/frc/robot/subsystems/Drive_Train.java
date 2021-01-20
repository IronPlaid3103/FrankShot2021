// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive_Train extends SubsystemBase {
  /** Creates a new Drive_Train. */
  public Drive_Train() {}

  private final WPI_TalonSRX _flDrive = new WPI_TalonSRX(Constants.DrivetrainConstants.flDrive);
  private final WPI_TalonSRX _frDrive = new WPI_TalonSRX(Constants.DrivetrainConstants.frDrive);
  private final WPI_TalonSRX _blDrive = new WPI_TalonSRX(Constants.DrivetrainConstants.blDrive);
  private final WPI_TalonSRX _brDrive = new WPI_TalonSRX(Constants.DrivetrainConstants.brDrive);

  private final MecanumDrive _drive = new MecanumDrive(_flDrive, _frDrive, _blDrive, _brDrive);

  public void teleopDrive(Joystick driver){
    double ySpeed = driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_X);
    double xSpeed = driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_Y);
    double zRotation = driver.getRawAxis(Constants.JoystickConstants.RIGHT_STICK_X);

    _drive.driveCartesian(ySpeed, -xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
