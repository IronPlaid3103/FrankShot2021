// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Drive_Train m_drivetrain = new Drive_Train();
  private final Joystick m_driver = new Joystick(0);
  private final Joystick m_operator = new Joystick(1); 
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();
  private final Shooter m_shooter = new Shooter();

  private SendableChooser<String> m_ChallengeChooser = new SendableChooser<String>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

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

    //in the parenthesis there is supposed to be (string, command) <-- in our case would it be (string, string)?

    SmartDashboard.putData("Starting Position", m_ChallengeChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_RIGHT);
    intakeButton.whileHeld(new IntakeIn(m_intake)); 

    JoystickButton hopperButton = new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_LEFT);
    hopperButton.whileHeld(new HooperGo(m_hopper));

    JoystickButton shooterButton = new JoystickButton(m_operator, Constants.JoystickConstants.A);
    shooterButton.whileHeld(new ShooterGo(m_shooter, m_hopper));

    JoystickButton driveRightButton = new JoystickButton(m_driver, Constants.JoystickConstants.A);
    driveRightButton.whenPressed(new AutonDriveRight(m_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //it says that it should be in here --- the problem is "cant convert string --> command"
    String challenge = m_ChallengeChooser.getSelected(); 

    String trajectoryJSON = "";
    if (challenge == "Galactic Search") {
      return new GalacticSearch(m_drivetrain, m_intake);
    } else if (challenge == "AutoNav - Barrel Racing") {
      trajectoryJSON = "AutoNav/PathWeaver/output/Barrel_Racing.wpilib.json";
    } else if (challenge == "AutoNav - Bounce") {
      trajectoryJSON = "AutoNav/PathWeaver/output/Bounce.wpilib.json";
    } else if (challenge == "AutoNav - Slalom") {
      trajectoryJSON = "AutoNav/PathWeaver/output/Slalom.wpilib.json";
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
}
