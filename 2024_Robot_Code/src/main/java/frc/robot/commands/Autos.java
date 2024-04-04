// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Launcher;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command centerAuto(DriveSubsystem drivesys, LauncherSubsystem launchsys, IntakeSubsystem intakesys, TurretSubsystem turretsys) {
    return Commands.sequence(
      launchsys.launchNote(intakesys, turretsys), 
      straightAutoCommand(drivesys, 3), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, 3),
      launchsys.launchNote(intakesys, turretsys));
  }

  /*public static Command leftAuto(DriveSubsystem drivesys, LauncherSubsystem launchsys, IntakeSubsystem intakesys, TurretSubsystem turretsys) {
    return Commands.sequence(launchsys.launchNote(intakesys, turretsys), 
      straightAutoCommand(drivesys, 3), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, -3),
      launchsys.launchNote(intakesys, turretsys));
  }

  public static Command rightAuto(DriveSubsystem drivesys, LauncherSubsystem launchsys, IntakeSubsystem intakesys, TurretSubsystem turretsys) {
    return Commands.sequence(launchsys.launchNote(intakesys, turretsys), 
      straightAutoCommand(drivesys, 3), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, -3),
      launchsys.launchNote(intakesys, turretsys));
  }*/

  public static Command straightAutoCommand(DriveSubsystem drivesys, double xpose) {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    edu.wpi.first.math.trajectory.Trajectory sTurnTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xpose/3, 0), new Translation2d(xpose*2/3, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xpose, 0, new Rotation2d(Math.toRadians(180))), config);

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    sTurnTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(sTurnTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


}
