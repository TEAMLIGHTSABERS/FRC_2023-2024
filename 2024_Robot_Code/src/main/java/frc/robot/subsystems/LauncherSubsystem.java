package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax lEChord;
  private CANSparkMax rEChord;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    lEChord =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        lEChord.setInverted(false);
        lEChord.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        lEChord.setIdleMode(IdleMode.kBrake);

        lEChord.burnFlash();

        rEChord =
        new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        rEChord.setInverted(true);
        rEChord.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        rEChord.setIdleMode(IdleMode.kBrake);

        rEChord.burnFlash();

    m_launcherRunning = false;
  }

/**
* Constructs a command that starts the launcher and plays/feeds a note by running the feed
* motor of the intake for after a short period of time. This command takes control of the
* intake subsystem to make sure the feeder keep running during the launch sequence.
*
* @param _intake The instance of the launcher subsystem
* @return The launch command
*/

public Command playNote(IntakeSubsystem _Intake) {
    Command harmony =
        new Command() {
          
          private Timer m_timer;

          @Override
          public void initialize() {
            m_launcherRunning = true;
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            if(m_timer.get() > Constants.Launcher.kPickupToMelody){
              _Intake.playMelody(Constants.Launcher.mezzoForte);
            }
          }

          @Override
          public boolean isFinished() {
            return m_timer.get() > Constants.Launcher.kMelodyOver;
          }

          @Override
          public void end(boolean interrupted) {
            _Intake.stopMusic();
            m_launcherRunning = false;
          }
        };

        return harmony;
  }
  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
//  public void runLauncher() {
//    m_launcherRunning = true;
//  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      rEChord.set(Constants.Launcher.kRightPower);
      lEChord.set(Constants.Launcher.kLeftPower);
    } else {
      rEChord.set(0.0);
      lEChord.set(0.0);
    }
  }
}
