package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private VictorSPX topRoller;
    private VictorSPX feedRollers;
    private double top_power;
    private double feed_power;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {

        topRoller = new VictorSPX(6);
        topRoller.setInverted(true);
        
        feedRollers = new VictorSPX(7);
        feedRollers.setInverted(true);

        top_power = 0.0;
        feed_power = 0.0;
        }
  /**
   * Set the power to spin the motor at. This only applies outside of position mode.
   *
   * @param _power The power to apply to the motor (from -1.0 to 1.0).
   */
  public void setPower(double _tpower, double _fpower) {
    top_power = _tpower;
    feed_power = _fpower;
  }

  /**
   * Constructs a command that feeds a note into the launcher by running the intake for a set amount
   * of time. This command takes control of the launcher subsystem to make sure the wheels keep
   * spinning during the launch sequence.
   *
   * @param _launcher The instance of the launcher subsystem
   * @return The launch command
   */
  public Command feedLauncher(LauncherSubsystem _launcher) {
    Command newCommand =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            setPower(1.0, 1.0);
            _launcher.runLauncher();
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Intake.kShotFeedTime;
          }

          @Override
          public void end(boolean interrupted) {
            setPower(0.0, 0.0);
          }
        };

    newCommand.addRequirements(this, _launcher);

    return newCommand;
  }

 

  @Override
  public void periodic() { // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    // update the motor power based on mode and setpoint
    topRoller.set(ControlMode.PercentOutput, top_power);
    feedRollers.set(ControlMode.PercentOutput, feed_power);
    
  }


}
