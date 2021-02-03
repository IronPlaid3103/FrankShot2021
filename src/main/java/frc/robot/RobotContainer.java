// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HooperGo;
import frc.robot.commands.HooperStop;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.Robot_Drive;
import frc.robot.commands.ShooterGo;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Drive_Train;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
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
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new Robot_Drive(m_drivetrain, m_driver));
    m_intake.setDefaultCommand(new IntakeStop(m_intake)); 
    m_hopper.setDefaultCommand(new HooperStop(m_hopper));
    m_shooter.setDefaultCommand(new ShooterStop(m_shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(m_operator, Constants.JoystickConstants.A);
    intakeButton.whileHeld(new IntakeIn(m_intake)); 

    JoystickButton hopperButton = new JoystickButton(m_operator, Constants.JoystickConstants.B);
    hopperButton.whileHeld(new HooperGo(m_hopper));

    JoystickButton shooterButton = new JoystickButton(m_operator, Constants.JoystickConstants.BUMPER_RIGHT);
    shooterButton.whileHeld(new ShooterGo(m_shooter));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ks, kv), kinematics, maxVoltage)

    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
