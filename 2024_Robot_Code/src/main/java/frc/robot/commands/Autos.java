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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

//  public static Command centerAuto(DriveSubsystem drivesys, LauncherSubsystem launchsys, IntakeSubsystem intakesys, TurretSubsystem turretsys) {
//    return Commands.sequence(straightAutoCommand(drivesys, -3, 0));
//  }

  public static Command centerAuto
  (
    DriveSubsystem driveSys,
    LauncherSubsystem launchSys,
    IntakeSubsystem intakeSys,
    TurretSubsystem turretSys
  )
  {
    return Commands.sequence(
      // Assumes that you align 0 heading along +x towards the opposing Alliance
      // and Robot get rotated to 180 degrees before the team leaves the field.
      // Starting position (0, 0, 180).
      raiseTurretCommand(turretSys, Constants.Turret.kHighShotID),   // Raise Turret to Speaker Shot Position
      launchNoteAuto(launchSys, intakeSys, turretSys),               // Shoot Note into Speaker
      straightAutoCommand11(driveSys),                               // move out of Zone to position (2, 0) m.                
      //straightAutoCommand12(driveSys),                               // move out of Zone to position (3, 0) m.                
      intakeSys.pickupNote(),                                        // Pickup the Note
      //straightAutoCommand13(driveSys),                               // move back to speaker (1, 0 180) m.                
      straightAutoCommand14(driveSys),                               // move up to Speaker (0, 0 180) m.                
      launchNoteAuto(launchSys, intakeSys, turretSys));              // Shoot Note into Speaker
  }

  /*public static Command leftAuto
  (
    DriveSubsystem drivesys,
    LauncherSubsystem launchsys,
    IntakeSubsystem intakesys,
    TurretSubsystem turretsys
  )
  {
    return Commands.sequence(
      // Assumes that you align 0 heading along +x towards the opposing Alliance
      // and Robot get rotated to 90 degrees to face the Amp before the team leaves the field.
      straightAutoCommand(drivesys, 1, 1, 90),                     // Drive to the Amp
  //    rotateBodyAzCommand(drivesys, 90),
      raiseTurretCommand(turretsys, Constants.Turret.kHighShotID), // Raise Turret to the Amp Position
      launchsys.launchNote(intakesys, turretsys),                  // Launch Note into the Amp
  //    rotateBodyAzCommand(drivesys, -90),
      straightAutoCommand(drivesys, 1, 1, 0),                      // Move out of Zone rotating to 0 
      straightAutoCommand(drivesys, 1, 0, 0),                      // Move to the Note
      intakesys.pickupNote(),                                      // PIckup Note
      straightAutoCommand(drivesys, -1, 90),                       //
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
      // Assumes that you align 0 heading along +x towards the opposing Alliance
      // and Robot get rotated to 180 degrees before the team leaves the field.
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
/*
  public static Command straightAutoCommand(DriveSubsystem drivesys, double xPos, double yPos, double finalAz) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double startAz = drivesys.getHeading();
    double heading = Units.radiansToDegrees(Math.atan2(xPos, -yPos));  // In Degrees

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // Determine if the Robot is moving in backward on the field.
    if (
      (Math.abs(startAz - heading) < 90.0)
      ||
      (Math.abs(startAz - heading)  > 270.0)
    ){
      // if the robot is moving forward along the direction of travel.
      config.setReversed(false);
    }else{
      // if the robot is moving in reverse along the direction of travel.
      config.setReversed(true);
    }

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(startAz))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(finalAz))), config
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
*/
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

public static Command launchNoteAuto(LauncherSubsystem _Launcher, IntakeSubsystem _Intake, TurretSubsystem _Turret) {
    Command launching =
        new Command() {
          
          private Timer m_timer;
          private int CurrentTurretPosition;
          //private double saveRightCmdRPM;
          //private double saveLeftCmdRPM;

          @Override
          public void initialize() {
            /* Start the Launcher Wheels and the Launch timer. */
            CurrentTurretPosition = _Turret.getSelPosition();

            if (CurrentTurretPosition == Constants.Turret.kAmpID) {
              _Launcher.SetLCWR(Constants.Launcher.kAmpL);
              _Launcher.SetRCWR(Constants.Launcher.kAmpR);
            }
            else if (CurrentTurretPosition == Constants.Turret.kHighShotID) {
              _Launcher.SetLCWR(Constants.Launcher.kHighShotL);
              _Launcher.SetRCWR(Constants.Launcher.kHighShotR);
            }
            else if (CurrentTurretPosition == Constants.Turret.kStartID) {
              _Launcher.SetLCWR(Constants.Launcher.kStartL);
              _Launcher.SetRCWR(Constants.Launcher.kStartR);
            }

            _Launcher.startFlyWheels();
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            /* Wait until the Launcher Wheels get up to speed,
             * then start the Intake Feeder to push the Note up
             * into the Launcher Wheels.
             */
            _Launcher.startFlyWheels();

             if(m_timer.get() > Constants.Launcher.kTimeToLaunch){
              _Intake.moveNote(Constants.Launcher.kFeederSpeed);
            }
          }

          @Override
          public boolean isFinished() {
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            _Launcher.stopFlyWheels();
            _Intake.stopFeeder();
          }
        };

        return launching;
  }


  public static Command straightAutoCommand1(DriveSubsystem drivesys, double xPos, double yPos) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(0))), config
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

  // version 11 move out of the zone by 2.5 m while turning from 180 degrees to 0 Degrees.
  public static Command straightAutoCommand11(DriveSubsystem drivesys) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double xPos = 2.0;
    double yPos = 0.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(xPos/3, yPos/3), new Translation2d(xPos*2/3, yPos*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(xPos, yPos, new Rotation2d(Math.toRadians(180))), config
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

  // version 12 move out of the zone by 1 m while turning from 180 degrees to 0 Degrees.
  public static Command straightAutoCommand12(DriveSubsystem drivesys) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double startXPos = 2.0;
    double startYPos = 0.0;
    double finishXPos = 3.0;
    double finishYPos = 0.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startXPos, startYPos, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d((finishXPos-startXPos)/3, (finishYPos-startYPos)/3),
                new Translation2d((finishXPos-startXPos)*2/3, (finishYPos-startYPos)*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(finishXPos, finishYPos, new Rotation2d(Math.toRadians(0))), config
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

  // version 13 moves back to the Speaker (1,0) while turning from 0 degrees to 180 Degrees.
  public static Command straightAutoCommand13(DriveSubsystem drivesys) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double startXPos = 3.0;
    double startYPos = 0.0;
    double finishXPos = 1.0;
    double finishYPos = 0.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startXPos, startYPos, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d((finishXPos-startXPos)/3, (finishYPos-startYPos)/3),
                new Translation2d((finishXPos-startXPos)*2/3, (finishYPos-startYPos)*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(finishXPos, finishYPos, new Rotation2d(Math.toRadians(180))), config
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

  // version 14 moves up to the Speaker launch position (0,0) 
  public static Command straightAutoCommand14(DriveSubsystem drivesys) {
    // create local parameters for the trajectory and robot angles
    edu.wpi.first.math.trajectory.Trajectory straightTrajectory;
    double startXPos = 2.0;
    double startYPos = 0.0;
    double finishXPos = 0.0;
    double finishYPos = 0.0; 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);

      // Create a straight trajectory to follow. All units in meters.
    straightTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startXPos, startYPos, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d((finishXPos-startXPos)/3, (finishYPos-startYPos)/3),
                new Translation2d((finishXPos-startXPos)*2/3, (finishYPos-startYPos)*2/3)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(finishXPos, finishYPos, new Rotation2d(Math.toRadians(180))), config
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

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


}
