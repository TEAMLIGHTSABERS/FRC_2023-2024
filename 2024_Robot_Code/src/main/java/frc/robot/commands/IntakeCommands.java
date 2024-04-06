// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

public final class IntakeCommands {
  private IntakeCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }


  /**
* Constructs a command that starts the Intake.
*
* @return The PickUp command
*/

/*
public Command pickupNoteAuto(IntakeSubsystem _Intake) {
    Command pickingUp =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
            _Intake.setTopRoller.set(ControlMode.PercentOutput, Constants.Intake.kTopPower);
            _Intake.setFeedRollers(ControlMode.PercentOutput, Constants.Intake.kFeedPower);
          }

          @Override
          public void execute() {
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Launcher.kTimeToStop;
          }

          @Override
          public void end(boolean interrupted) {
            topRoller.set(ControlMode.PercentOutput, 0.0);
            feedRollers.set(ControlMode.PercentOutput, 0.0);
          }
        };

        return pickingUp;
  }
*/

}
