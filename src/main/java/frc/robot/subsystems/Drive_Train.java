// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive_Train extends SubsystemBase {
  private final CANSparkMax _flDrive = new CANSparkMax(Constants.DrivetrainConstants.flDrive, MotorType.kBrushless);
  private final CANSparkMax _frDrive = new CANSparkMax(Constants.DrivetrainConstants.frDrive, MotorType.kBrushless);
  private final CANSparkMax _blDrive = new CANSparkMax(Constants.DrivetrainConstants.blDrive, MotorType.kBrushless);
  private final CANSparkMax _brDrive = new CANSparkMax(Constants.DrivetrainConstants.brDrive, MotorType.kBrushless);

  private final MecanumDrive _drive = new MecanumDrive(_flDrive, _frDrive, _blDrive, _brDrive);

  /** Creates a new Drive_Train. */
  public Drive_Train() {
    _drive.setDeadband(Constants.DrivetrainConstants.deadband); 
    //if mecanum drive doesnt work then add own deadband function
  }

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
