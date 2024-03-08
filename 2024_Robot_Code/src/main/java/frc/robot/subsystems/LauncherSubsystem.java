package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax leftLaunchWheel;
  private CANSparkMax rightLaunchWheel;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    leftLaunchWheel =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        leftLaunchWheel.setInverted(false);
        leftLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        leftLaunchWheel.setIdleMode(IdleMode.kBrake);

        leftLaunchWheel.burnFlash();

        rightLaunchWheel =
        new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        rightLaunchWheel.setInverted(true);
        rightLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        rightLaunchWheel.setIdleMode(IdleMode.kBrake);

        rightLaunchWheel.burnFlash();

    m_launcherRunning = false;
  }

/**
* Constructs a command that starts the launcher and then runs the Intake feeder
* motor to put the Note up to the spinning launcher wheels. After a few more seconds
* the command will shutdown the Launcher Wheels and an the Intake feeder. This
* command takes control of the intake subsystem to make sure the feeder keeps
* running during the launch sequence.
*
* @param _intake The instance of the launcher subsystem
* @return The launch command
*/

public Command launchNote(IntakeSubsystem _Intake) {
    Command launching =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            /* Start the Launcher Wheels and the Launch timer. */
            m_launcherRunning = true;
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            /* Wait until the Launcher Wheels get up to speed,
             * then start the Intake Feeder to push the Note up
             * into the Launcher Wheels.
             */
            m_launcherRunning = true;
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
            m_launcherRunning = false;
            _Intake.stopFeeder();
          }
        };

        return launching;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  /* Return the current power on the left Launcher wheel. */
  public double getLeftLauchPower(){
     return (Constants.Launcher.kLeftPower);
  }

  /* Return the current power on the right Launcher wheel. */
  public double getRightLauchPower(){
     return (Constants.Launcher.kRightPower);
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      rightLaunchWheel.set(Constants.Launcher.kRightPower);
      leftLaunchWheel.set(Constants.Launcher.kLeftPower);
    } else {
      rightLaunchWheel.set(0.0);
      leftLaunchWheel.set(0.0);
    }
  }
}
