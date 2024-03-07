package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private CANSparkMax LTShooter;
  private CANSparkMax RTShooter;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    LTShooter =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        LTShooter.setInverted(false);
        LTShooter.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        LTShooter.setIdleMode(IdleMode.kBrake);

        LTShooter.burnFlash();

        RTShooter =
        new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);
        RTShooter.setInverted(true);
        RTShooter.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
        RTShooter.setIdleMode(IdleMode.kBrake);

        RTShooter.burnFlash();

    m_launcherRunning = false;
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

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
      RTShooter.set(Constants.Launcher.kRightPower);
      LTShooter.set(Constants.Launcher.kLeftPower);
    } else {
      RTShooter.set(0.0);
      LTShooter.set(0.0);
    }
  }
}
