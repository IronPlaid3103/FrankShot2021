// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;

public class Drive_Train extends SubsystemBase {
  private final CANSparkMax _flDrive = new CANSparkMax(Constants.DrivetrainConstants.flDrive, MotorType.kBrushless);
  private final CANSparkMax _frDrive = new CANSparkMax(Constants.DrivetrainConstants.frDrive, MotorType.kBrushless);
  private final CANSparkMax _blDrive = new CANSparkMax(Constants.DrivetrainConstants.blDrive, MotorType.kBrushless);
  private final CANSparkMax _brDrive = new CANSparkMax(Constants.DrivetrainConstants.brDrive, MotorType.kBrushless);

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  private final MecanumDrive _drive = new MecanumDrive(_flDrive, _frDrive, _blDrive, _brDrive);

  public CANEncoder FREncoder;
  public CANEncoder FLEncoder;

  public Encoder m_left_follower; //encoder follow in old code dont know what means
  public Encoder m_right_follower;


  /** Creates a new Drive_Train. */
  public Drive_Train() {
    _drive.setDeadband(Constants.DrivetrainConstants.deadband); 
    //if mecanum drive doesnt work then add own deadband function
  }

  public void teleopDrive(Joystick driver){
    double ySpeed = driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_X);
    double xSpeed = driver.getRawAxis(Constants.JoystickConstants.LEFT_STICK_Y);
    double zRotation = driver.getRawAxis(Constants.JoystickConstants.RIGHT_STICK_X);

    _drive.driveCartesian(ySpeed, -xSpeed, zRotation, gyro.getAngle());
  }

  public void encoderReset(){
    FREncoder.setPosition(0);
    FLEncoder.setPosition(0);
  }

  public boolean pathAuton(String pathname){
    encoderReset();

    Trajectory left_trajectory;
    Trajectory right_trajectory;

    String trajectoryJSON = "paths/hi.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException ex){
      DriverStation.reportError("Unable to open Trajectory" + trajectoryJSON, ex.getStackTrace());
    }
    
    // implement file exists code

    //left_trajectory = PathfinderFRC.getTrajectory(pathname + ".right");
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
