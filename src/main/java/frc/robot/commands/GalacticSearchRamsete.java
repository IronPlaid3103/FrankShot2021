// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive_Train;

public class GalacticSearchRamsete extends CommandBase {
  private Drive_Train _drivetrain;
  private RamseteCommand _ramsete;

  /** Creates a new GalacticSearchRamsete. */
  public GalacticSearchRamsete(Drive_Train drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_ramsete == null) {
      _ramsete = createRamseteCommand();
      _ramsete.initialize();
      return;
    }

    _ramsete.execute();
  }

  private RamseteCommand createRamseteCommand() {
    //stuff

    NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable("Frank").getEntry("GalacticSearchPath");
    String path = entry.getString("");

    String trajectoryJSON = "";

    if (path == "ARed") {
      trajectoryJSON = "Galactic_Search_A/PathWeaver/output/Red.wpilib.json";
    } else if (path == "ABlue") {
      trajectoryJSON = "Galactic_Search_A/PathWeaver/output/Blue.wpilib.json";
    } else if (path == "BRed") {
      trajectoryJSON = "Galactic_Search_B/PathWeaver/output/Red.wpilib.json";
    } else if (path == "BBlue") {
      trajectoryJSON = "Galactic_Search_B/PathWeaver/output/Blue.wpilib.json";
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
        _drivetrain::getPose,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
            Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
            Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DrivetrainConstants.kDriveKinematics, _drivetrain::getWheelSpeeds,
        new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        _drivetrain::tankDriveVolts, _drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    _drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _ramsete.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _ramsete.isFinished();
  }
}
