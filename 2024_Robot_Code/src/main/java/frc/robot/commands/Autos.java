// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command centerAuto(DriveSubsystem drivesys, LauncherSubsystem launchsys, IntakeSubsystem intakesys, TurretSubsystem turretsys) {
    return Commands.sequence(straightAutoCommand(drivesys, -3, 0));
  }

  /*public static Command centerAuto
  (
    DriveSubsystem drivesys,
    LauncherSubsystem launchsys,
    IntakeSubsystem intakesys,
    TurretSubsystem turretsys
  )
  {
    return Commands.sequence(
      raiseTurretCommand(turretsys, Constants.Turret.kHighShotID)
      launchsys.launchNote(intakesys, turretsys), 
      straightAutoCommand(drivesys, 3), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, 3),
      launchsys.launchNote(intakesys, turretsys));
  }*/

  /*public static Command leftAuto
  (
    DriveSubsystem drivesys,
    LauncherSubsystem launchsys,
    IntakeSubsystem intakesys,
    TurretSubsystem turretsys
  )
  {
    return Commands.sequence(
      straightAutoCommand(drivesys, 1, -1), 
      rotateBodyAzCommand(drivesys, 90)
      raiseTurretCommand(turretsys, Constants.Turret.kHighShotID)
      launchsys.launchNote(intakesys, turretsys), 
      rotateBodyAzCommand(drivesys, -90)
      straightAutoCommand(drivesys, 0, 1), 
      straightAutoCommand(drivesys, 2, 0), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, -2, -1),
      rotateBodyAzCommand(drivesys, 90)
      launchsys.launchNote(intakesys, turretsys));
  }*/

  /*public static Command RightAuto
  (
    DriveSubsystem drivesys,
    LauncherSubsystem launchsys,
    IntakeSubsystem intakesys,
    TurretSubsystem turretsys
  )
  {
    return Commands.sequence(
      rotateBodyAzCommand(drivesys, 45)
      raiseTurretCommand(turretsys, Constants.Turret.kAmpID)
      launchsys.launchNote(intakesys, turretsys), 
      rotateBodyAzCommand(drivesys, -45)
      straightAutoCommand(drivesys, 3), 
      intakesys.pickupNote(), 
      straightAutoCommand(drivesys, 3),
      rotateBodyAzCommand(drivesys, 45),
      launchsys.launchNote(intakesys, turretsys));
  }*/

  public static Command straightAutoCommand(DriveSubsystem drivesys, double deltaXPos, double deltaYPos) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double azimuth = drivesys.getHeading();
    double heading = Units.radiansToDegrees(Math.atan2(deltaXPos, -deltaYPos));  // In Degrees

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Determine if the Robot is moving in backward on the field.
    if (Math.abs(azimuth - heading) < 90.0){
      // if the robot is moving forward along the direction of travel.
      config.setReversed(false);
    }else{
      // if the robot is moving in reverse along the direction of travel.
      config.setReversed(true);
    }

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(heading))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(deltaXPos/3, deltaYPos/3), new Translation2d(deltaXPos*2/3, deltaYPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(deltaXPos, deltaYPos, new Rotation2d(Math.toRadians(heading))), config
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    straightTrajectory,
    drivesys::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    drivesys::setModuleStates,
    drivesys);

    // Reset odometry to the starting pose of the trajectory.
    drivesys.resetOdometry(straightTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivesys.drive(0, 0, 0, false, false));
  }

  public static Command rotateBodyAzCommand(DriveSubsystem drivesys, double cmdedFieldAz) {
    Command rotateTo =
        new Command() {
          
          @Override
          public void initialize() {
          }

          @Override
          public void execute() {
            double currAz = drivesys.getHeading();
            
            if (currAz < cmdedFieldAz){
              drivesys.drive(0.0, 0.0, 0.1, true, false);
            } else {
              drivesys.drive(0.0, 0.0, -0.1, true, false);
            }
          }

          @Override
          public boolean isFinished() {
            double currAz = drivesys.getHeading();
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            Boolean reachedDeadband = Math.abs
              (
                MathUtil.applyDeadband
                (
                  currAz - cmdedFieldAz,
                  Constants.OIConstants.kDriveAngleDeadband
                )
              ) < Constants.OIConstants.kDriveAngleDeadband;

            return (reachedDeadband);
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            drivesys.drive(0.0, 0.0, 0.0, true, false);
          }
        };

    return rotateTo;
  }

  public static Command raiseTurretCommand(TurretSubsystem turretSys, int cmdedTurretPos) {
    Command raiseTo =
        new Command() {
          
          @Override
          public void initialize() {

            int currSelPos = turretSys.getSelPosition();
            while(currSelPos != cmdedTurretPos)
            {
              if (currSelPos < cmdedTurretPos) {
                turretSys.advancePOS();
              } else if (currSelPos > cmdedTurretPos) {
                turretSys.reducePOS();
              }

              currSelPos = turretSys.getSelPosition();
            };
          }

          @Override
          public boolean isFinished() {
            return (true);
          }
        };

    return raiseTo;
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


}
