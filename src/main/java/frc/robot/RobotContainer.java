// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.COLOR;
import frc.robot.util.LIDARLiteV3;
import frc.robot.util.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final LIDARLiteV3 m_lidar = new LIDARLiteV3(0,0);
  private final Drive_Train m_drivetrain = new Drive_Train(m_gyro);
  private final Joystick m_driver = new Joystick(0);
  private final Joystick m_operator = new Joystick(1); 
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();
  private final Limelight m_limelight = new Limelight();

  private SendableChooser<String> m_ChallengeChooser = new SendableChooser<String>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    loadSettings();

    m_drivetrain.setDefaultCommand(new Robot_Drive(m_drivetrain, m_driver));
    m_intake.setDefaultCommand(new IntakeStop(m_intake)); 
    m_hopper.setDefaultCommand(new HooperStop(m_hopper));
    // m_shooter.setDefaultCommand(new ShooterStop(m_shooter));

    //setting up SendableChooser
    m_ChallengeChooser = new SendableChooser<>();
    m_ChallengeChooser.setDefaultOption("Galactic Search", "Galactic Search");
    m_ChallengeChooser.addOption("AutoNav - Barrel Racing", "AutoNav - Barrel Racing");
    m_ChallengeChooser.addOption("AutoNav - Bounce", "AutoNav - Bounce");
    m_ChallengeChooser.addOption("AutoNav - Slalom", "AutoNav - Slalom");
    m_ChallengeChooser.addOption("Test-Straight", "Test-Straight");
    m_ChallengeChooser.addOption("Test-Turn", "Test-Turn");
    m_ChallengeChooser.addOption("Test-Curve", "Test-Curve");

    SmartDashboard.putData("Challenge Chooser", m_ChallengeChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //OPERATOR
    new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeIn(m_intake)); 
    new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(m_intake)); 

    new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new HooperGo(m_hopper));
    new JoystickButton(m_operator, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new HopperBack(m_hopper));

    new JoystickButton(m_operator, Constants.JoystickConstants.A).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper)); 
    new JoystickButton(m_operator, Constants.JoystickConstants.Y).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper));
    new JoystickButton(m_operator, Constants.JoystickConstants.X).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper));
    new JoystickButton(m_operator, Constants.JoystickConstants.B).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper));

    //DRIVER
    new JoystickButton(m_driver, Constants.JoystickConstants.LOGO_LEFT).whenPressed(new InstantCommand(() -> m_gyro.reset()));

    new JoystickButton(m_driver, Constants.JoystickConstants.A).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Green)); 
    new JoystickButton(m_driver, Constants.JoystickConstants.Y).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Yellow));
    new JoystickButton(m_driver, Constants.JoystickConstants.X).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Blue));
    new JoystickButton(m_driver, Constants.JoystickConstants.B).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Red));

    new JoystickButton(m_driver, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new LimelightAim(m_drivetrain, m_limelight));

    // new JoystickButton(m_driver, Constants.JoystickConstants.LOGO_RIGHT).whenPressed(new AutonDriveRight(m_drivetrain, m_gyro, m_lidar));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String challenge = m_ChallengeChooser.getSelected(); 

    String trajectoryJSON = "";
    if (challenge == "Galactic Search") {
      return new GalacticSearch(m_drivetrain, m_intake, m_gyro, m_lidar);
    } else if (challenge == "AutoNav - Barrel Racing") {
      trajectoryJSON = "Paths/output/AutoNav--Barrel Racing.wpilib.json";
    } else if (challenge == "AutoNav - Bounce") {
      trajectoryJSON = "Paths/output/AutoNav--Bounce.wpilib.json";
    } else if (challenge == "AutoNav - Slalom") {
      trajectoryJSON = "Paths/output/AutoNav--Slalom.wpilib.json";
    } else if (challenge == "Test-Straight") {
      trajectoryJSON = "Paths/output/test-straight.wpilib.json";
    } else if (challenge == "Test-Turn") {
      trajectoryJSON = "Paths/output/test-turn.wpilib.json";
    } else if (challenge == "Test-Curve") {
      trajectoryJSON = "Paths/output/test-curve.wpilib.json";
    }

    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrain::getPose,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
          Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
          Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DrivetrainConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }

  public void loadSettings(){
    m_intake.setPower(Settings.loadDouble("Intake", "Power", Constants.IntakeConstants.defaultPower));
    m_hopper.setPower(Settings.loadDouble("Hopper", "Power", Constants.HopperConstants.defaultPower));
    m_shooter.setkP(Settings.loadDouble("Shooter", "kF", Constants.ShooterConstants.defaultkF));
    m_shooter.setkF(Settings.loadDouble("Shooter", "kP", Constants.ShooterConstants.defaultkP));
    m_shooter.setRedVelocity(Settings.loadDouble("Shooter", "RedVelocity", Constants.ShooterConstants.redVelocity));
    m_shooter.setBlueVelocity(Settings.loadDouble("Shooter", "BlueVelocity", Constants.ShooterConstants.blueVelocity));
    m_shooter.setYellowVelocity(Settings.loadDouble("Shooter", "YellowVelocity", Constants.ShooterConstants.yellowVelocity));
    m_shooter.setGreenVelocity(Settings.loadDouble("Shooter", "GreenVelocity", Constants.ShooterConstants.greenVelocity));
    m_drivetrain.setkP(Settings.loadDouble("Limelight", "kP", Constants.LimelightConstants.kP));
    m_drivetrain.setkI(Settings.loadDouble("Limelight", "kI", Constants.LimelightConstants.kI));
    m_drivetrain.setkD(Settings.loadDouble("Limelight", "kD", Constants.LimelightConstants.kD));
  }

  public void saveSettings(){
    Settings.saveDouble("Intake", "Power", m_intake.getPower());
    Settings.saveDouble("Hopper", "Power", m_hopper.getPower());
    Settings.saveDouble("Shooter", "kF", m_shooter.getkF());
    Settings.saveDouble("Shooter", "kP", m_shooter.getkP());
    Settings.saveDouble("Shooter", "RedVelocity", m_shooter.getRedVelocity());
    Settings.saveDouble("Shooter", "BlueVelocity", m_shooter.getBlueVelocity());
    Settings.saveDouble("Shooter", "YellowVelocity", m_shooter.getYellowVelocity());
    Settings.saveDouble("Shooter", "GreenVelocity", m_shooter.getGreenVelocity());
    Settings.saveDouble("Limelight", "kP", m_drivetrain.getkP());
    Settings.saveDouble("Limelight", "kI", m_drivetrain.getkI());
    Settings.saveDouble("Limelight", "kD", m_drivetrain.getkD());
  }
}
