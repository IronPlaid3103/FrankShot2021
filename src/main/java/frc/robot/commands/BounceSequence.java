// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drive_Train;
import frc.robot.util.TrajectoryCache;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceSequence extends SequentialCommandGroup {
  /** Creates a new BounceSequence. */
  public BounceSequence(Drive_Train drivetrain) {
    RamseteController ramseteController = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kvVoltSecondsPerMeter, DrivetrainConstants.kaVoltSecondsSquaredPerMeter);
    PIDController pidLeft = new PIDController(DrivetrainConstants.kPDriveVel, 0, 0);
    PIDController pidRight = new PIDController(DrivetrainConstants.kPDriveVel, 0, 0);

    Trajectory trajectory0 = TrajectoryCache.get("AutoNav--Bounce0");
    RamseteCommand ramsete0 = new RamseteCommand(trajectory0, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.kDriveKinematics, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory1 = TrajectoryCache.get("AutoNav--Bounce1");
    RamseteCommand ramsete1 = new RamseteCommand(trajectory1, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.kDriveKinematics, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory2 = TrajectoryCache.get("AutoNav--Bounce2");
    RamseteCommand ramsete2 = new RamseteCommand(trajectory2, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.kDriveKinematics, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);
    Trajectory trajectory3 = TrajectoryCache.get("AutoNav--Bounce3");
    RamseteCommand ramsete3 = new RamseteCommand(trajectory3, drivetrain::getPose, ramseteController, feedforward, DrivetrainConstants.kDriveKinematics, drivetrain::getWheelSpeeds, pidLeft, pidRight, drivetrain::tankDriveVolts, drivetrain);

    addCommands(
      ramsete0.beforeStarting(() -> drivetrain.resetOdometry(trajectory0.getInitialPose()), drivetrain),
      ramsete1.beforeStarting(() -> drivetrain.resetOdometry(trajectory1.getInitialPose()), drivetrain),
      ramsete2.beforeStarting(() -> drivetrain.resetOdometry(trajectory2.getInitialPose()), drivetrain),
      ramsete3.beforeStarting(() -> drivetrain.resetOdometry(trajectory3.getInitialPose()), drivetrain)
    );
  }
}
