// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.analog.adis16470.frc.ADIS16470_IMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
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
    loadChallengeChooser();
  }

  private void loadChallengeChooser() {
    TrajectoryCache.clear();
    m_ChallengeChooser = new SendableChooser<>();
    m_ChallengeChooser.addOption("Galactic Search", "Galactic Search");

    cacheTrajectory("AutoNav - Barrel Racing", "Paths/output/AutoNav--Barrel_Racing.wpilib.json");
    m_ChallengeChooser.addOption("AutoNav - Bounce", "AutoNav - Bounce");
    cacheTrajectory("AutoNav - Slalom", "Paths/output/AutoNav--Slalom.wpilib.json");

    cacheTrajectory("GS A Blue", "Paths/output/GS_A--Blue.wpilib.json");
    cacheTrajectory("GS A Red", "Paths/output/GS_A--Red.wpilib.json");
    cacheTrajectory("GS B Blue", "Paths/output/GS_B--Blue.wpilib.json");
    cacheTrajectory("GS B Red", "Paths/output/GS_B--Red.wpilib.json");

    cacheTrajectory("AutoNav--Bounce0", "Paths/output/AutoNav--Bounce0.wpilib.json");
    cacheTrajectory("AutoNav--Bounce1", "Paths/output/AutoNav--Bounce1.wpilib.json");
    cacheTrajectory("AutoNav--Bounce2", "Paths/output/AutoNav--Bounce2.wpilib.json");
    cacheTrajectory("AutoNav--Bounce3", "Paths/output/AutoNav--Bounce3.wpilib.json");

    cacheTrajectory("Test-Straight", "Paths/output/test-straight.wpilib.json");
    cacheTrajectory("Test-Turn", "Paths/output/test-turn.wpilib.json");
    cacheTrajectory("Test-Turn2", "Paths/output/test-turn2.wpilib.json");
    cacheTrajectory("Test-Curve", "Paths/output/test-curve.wpilib.json");
    cacheTrajectory("Test-StraightReverse", "Paths/output/test-straightreverse.wpilib.json");
    
    m_ChallengeChooser.addOption("Test-Group", "Test-Group");

    SmartDashboard.putData("Challenge Chooser", m_ChallengeChooser);
  }

  private void cacheTrajectory(String key, String trajectoryJson) {
    m_ChallengeChooser.addOption(key, key);
    TrajectoryCache.add(key, trajectoryJson);
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

    new JoystickButton(m_operator, Constants.JoystickConstants.A).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper, COLOR.Green)); 
    new JoystickButton(m_operator, Constants.JoystickConstants.Y).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper, COLOR.Yellow));
    new JoystickButton(m_operator, Constants.JoystickConstants.X).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper, COLOR.Blue));
    new JoystickButton(m_operator, Constants.JoystickConstants.B).whileHeld(new AimAndShoot(m_drivetrain, m_limelight, m_shooter, m_hopper, COLOR.Red));

    new JoystickButton(m_operator, Constants.JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> m_limelight.toggleBypass()));

    //DRIVER
    new JoystickButton(m_driver, Constants.JoystickConstants.LOGO_LEFT).whenPressed(new InstantCommand(() -> m_gyro.reset()));

    new JoystickButton(m_driver, Constants.JoystickConstants.A).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Green)); 
    new JoystickButton(m_driver, Constants.JoystickConstants.Y).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Yellow));
    new JoystickButton(m_driver, Constants.JoystickConstants.X).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Blue));
    new JoystickButton(m_driver, Constants.JoystickConstants.B).whileHeld(new ShooterGo(m_shooter, m_hopper, COLOR.Red));

    new JoystickButton(m_driver, Constants.JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(() -> loadChallengeChooser()));

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

    Trajectory trajectory;
    if (challenge == "Galactic Search") {
      return new GalacticSearch(m_drivetrain, m_intake, m_gyro, m_lidar);
    } else if (challenge == "AutoNav - Bounce") {
      return new BounceSequence(m_drivetrain);
    } else if (challenge == "Test-Group") {
      return new TestGroupSequence(m_drivetrain);
    } else {
      trajectory = TrajectoryCache.get(challenge);
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
    m_hopper.setFeederPower(Settings.getLiveDouble("Hopper", "FeederPower", Constants.HopperConstants.hopperFeederPower));
    m_shooter.setkP(Settings.loadDouble("Shooter", "kF", Constants.ShooterConstants.defaultkF));
    m_shooter.setkF(Settings.loadDouble("Shooter", "kP", Constants.ShooterConstants.defaultkP));
    m_shooter.setRedVelocity(Settings.loadDouble("Shooter", "RedVelocity", Constants.ShooterConstants.redVelocity));
    m_shooter.setBlueVelocity(Settings.loadDouble("Shooter", "BlueVelocity", Constants.ShooterConstants.blueVelocity));
    m_shooter.setYellowVelocity(Settings.loadDouble("Shooter", "YellowVelocity", Constants.ShooterConstants.yellowVelocity));
    m_shooter.setGreenVelocity(Settings.loadDouble("Shooter", "GreenVelocity", Constants.ShooterConstants.greenVelocity));
    m_drivetrain.setkP(Settings.loadDouble("Limelight", "kP", Constants.LimelightConstants.kP));
    m_drivetrain.setkI(Settings.loadDouble("Limelight", "kI", Constants.LimelightConstants.kI));
    m_drivetrain.setkD(Settings.loadDouble("Limelight", "kD", Constants.LimelightConstants.kD));
    m_drivetrain.setksVolts(Settings.loadDouble("DriveTrain", "ksVolts", Constants.DrivetrainConstants.ksVolts));
    m_drivetrain.setkvVoltSecondsPerMeter(Settings.loadDouble("DriveTrain", "kvVoltSecondsPerMeter", Constants.DrivetrainConstants.kvVoltSecondsPerMeter));
    m_drivetrain.setkaVoltSecondsSquaredPerMeter(Settings.loadDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
  }

  public void saveSettings(){
    Settings.saveDouble("Intake", "Power", m_intake.getPower());
    Settings.saveDouble("Hopper", "Power", m_hopper.getPower());
    Settings.saveDouble("Hopper", "FeederPower", m_hopper.getFeederPower());
    Settings.saveDouble("Shooter", "kF", m_shooter.getkF());
    Settings.saveDouble("Shooter", "kP", m_shooter.getkP());
    Settings.saveDouble("Shooter", "RedVelocity", m_shooter.getRedVelocity());
    Settings.saveDouble("Shooter", "BlueVelocity", m_shooter.getBlueVelocity());
    Settings.saveDouble("Shooter", "YellowVelocity", m_shooter.getYellowVelocity());
    Settings.saveDouble("Shooter", "GreenVelocity", m_shooter.getGreenVelocity());
    Settings.saveDouble("Limelight", "kP", m_drivetrain.getkP());
    Settings.saveDouble("Limelight", "kI", m_drivetrain.getkI());
    Settings.saveDouble("Limelight", "kD", m_drivetrain.getkD());
    Settings.saveDouble("DriveTrain", "ksVolts", m_drivetrain.getksVolts());
    Settings.saveDouble("DriveTrain", "kvVoltsSecondsPerMeter", m_drivetrain.getkvVoltSecondsPerMeter());
    Settings.saveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", m_drivetrain.getkaVoltSecondsSquaredPerMeter());
  }
}
