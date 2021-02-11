// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Robot_Drive(m_drivetrain, m_driver));
    m_intake.setDefaultCommand(new IntakeStop(m_intake)); 
    m_hopper.setDefaultCommand(new HooperStop(m_hopper));
    // m_shooter.setDefaultCommand(new ShooterStop(m_shooter));
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
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
                                   Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                                   Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DrivetrainConstants.kDriveKinematics,
        10);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.DrivetrainConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config
);

// Trajectory traj = exampleTrajectory;
// RamseteController ramsete = new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta);
// SimpleMotorFeedforward smff = new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
// Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
// Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter);
// PIDController pid1 = new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0);
// PIDController pid2 = new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0);
// RamseteCommand ramseteCommand = new RamseteCommand(
//     traj,
//     m_drivetrain::getPose,
//     ramsete,
//     smff,
//     Constants.DrivetrainConstants.kDriveKinematics,
//     m_drivetrain::getWheelSpeeds,
//     pid1,
//     pid2,
//     // RamseteCommand passes volts to the callback
//     m_drivetrain::tankDriveVolts,
//     m_drivetrain
// );

// Reset odometry to the starting pose of the trajectory.
m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return null;/ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
  }
}
