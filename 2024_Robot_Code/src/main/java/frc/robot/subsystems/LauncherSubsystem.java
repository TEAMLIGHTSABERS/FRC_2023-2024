package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  public double kMaxOutput, kMinOutput, maxRPM;

  private double leftSetPoint; 

  private double rightSetPoint;

  private CANSparkMax leftLaunchWheel;
  private CANSparkMax rightLaunchWheel;

  private SparkPIDController m_leftlancherpidCtrl;
  private RelativeEncoder m_Lencoder;
  public double kLP, kLI, kLD, kLIz, kLFF ;


  private SparkPIDController m_rightlancherpidCtrl;
  private RelativeEncoder m_Rencoder;
  public double kRP, kRI, kRD, kRIz, kRFF ;

  private boolean m_launcherRunning;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    leftLaunchWheel =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Add Left Launch Wheel motor to the LiveWindow.
    addChild("Left Launch Wheel", leftLaunchWheel);
        
    leftLaunchWheel.setInverted(false);
    leftLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    leftLaunchWheel.setIdleMode(IdleMode.kBrake);
    
    m_leftlancherpidCtrl = leftLaunchWheel.getPIDController();

    // Encoder object created to display position values
    m_Lencoder = leftLaunchWheel.getEncoder();

    // PID coefficients
    kLP = 6e-5; 
    kLI = 0;
    kLD = 0; 
    kLIz = 0; 
    kLFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_leftlancherpidCtrl.setP(kLP);
    m_leftlancherpidCtrl.setI(kLI);
    m_leftlancherpidCtrl.setD(kLD);
    m_leftlancherpidCtrl.setIZone(kLIz);
    m_leftlancherpidCtrl.setFF(kLFF);
    m_leftlancherpidCtrl.setOutputRange(kMinOutput, kMaxOutput);


    leftLaunchWheel.burnFlash();

    rightLaunchWheel =
    new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Add Right Launch Wheel motor to the LiveWindow.
    addChild("Right Launch Wheel", rightLaunchWheel);

    rightLaunchWheel.setInverted(true);
    rightLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    rightLaunchWheel.setIdleMode(IdleMode.kBrake);

    m_rightlancherpidCtrl = rightLaunchWheel.getPIDController();

    // Encoder object created to display position values
    m_Rencoder = rightLaunchWheel.getEncoder();

    // PID coefficients
    kRP = 6e-5; 
    kRI = 0;
    kRD = 0; 
    kRIz = 0; 
    kRFF = 0.000015; 

    // set PID coefficients
    m_rightlancherpidCtrl.setP(kRP);
    m_rightlancherpidCtrl.setI(kRI);
    m_rightlancherpidCtrl.setD(kRD);
    m_rightlancherpidCtrl.setIZone(kRIz);
    m_rightlancherpidCtrl.setFF(kRFF);
    m_rightlancherpidCtrl.setOutputRange(kMinOutput, kMaxOutput);

    rightLaunchWheel.burnFlash();

    m_launcherRunning = false;


  
  }

private void addChild(String name, CANSparkMax leftLaunchWheel2) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'addChild'");
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
  public double getLeftLaunchPower(){
    if(m_launcherRunning)
    {
     return (Constants.Launcher.kLeftPower);
    }
    else
    {
     return (0.0);
    }
  }

/* Set the right launcher speed */
  public void setRightLaunchSpeed(double _rightSetPoint){
    rightSetPoint = _rightSetPoint; //1500  //rpm
  }

  /* Set the right launcher speed */
  public void setLeftLaunchSpeed(double _leftSetPoint){
    leftSetPoint = _leftSetPoint; //1500  //rpm
  }

  /* Return the current power on the right Launcher wheel. */
  public double getRightLaunchPower(){
    if(m_launcherRunning)
    {
     return (Constants.Launcher.kRightPower);
    }
    else
    {
     return (0.0);
    }
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not

// read PID coefficients from SmartDashboard
    double lp = SmartDashboard.getNumber("P Gain", 0);
    double li = SmartDashboard.getNumber("I Gain", 0);
    double ld = SmartDashboard.getNumber("D Gain", 0);
    double liz = SmartDashboard.getNumber("I Zone", 0);
    double lff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((lp != kLP)) { m_leftlancherpidCtrl.setP(lp); kLP = lp; }
    if((li != kLI)) { m_leftlancherpidCtrl.setI(li); kLI = li; }
    if((ld != kLD)) { m_leftlancherpidCtrl.setD(ld); kLD = ld; }
    if((liz != kLIz)) { m_leftlancherpidCtrl.setIZone(liz); kLIz = liz; }
    if((lff != kLFF)) { m_leftlancherpidCtrl.setFF(lff); kLFF = lff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_leftlancherpidCtrl.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    // read PID coefficients from SmartDashboard
    double rp = SmartDashboard.getNumber("P Gain", 0);
    double ri = SmartDashboard.getNumber("I Gain", 0);
    double rd = SmartDashboard.getNumber("D Gain", 0);
    double riz = SmartDashboard.getNumber("I Zone", 0);
    double rff = SmartDashboard.getNumber("Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((rp != kRP)) { m_leftlancherpidCtrl.setP(rp); kRP = rp; }
    if((ri != kRI)) { m_leftlancherpidCtrl.setI(ri); kRI = ri; }
    if((rd != kRD)) { m_leftlancherpidCtrl.setD(rd); kRD = rd; }
    if((riz != kRIz)) { m_leftlancherpidCtrl.setIZone(riz); kRIz = riz; }
    if((rff != kRFF)) { m_leftlancherpidCtrl.setFF(rff); kRFF = rff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_rightlancherpidCtrl.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    if (m_launcherRunning) {
      m_rightlancherpidCtrl.setReference(rightSetPoint, CANSparkMax.ControlType.kVelocity);
      m_leftlancherpidCtrl.setReference(leftSetPoint, CANSparkMax.ControlType.kVelocity);

      //rightLaunchWheel.set(Constants.Launcher.kRightPower);
      //leftLaunchWheel.set(Constants.Launcher.kLeftPower);

    } else {
      rightLaunchWheel.set(0.0);
      leftLaunchWheel.set(0.0);
    }
    SmartDashboard.putNumber("RightSetPoint", rightSetPoint);
    SmartDashboard.putNumber("LeftSetPoint", leftSetPoint);
    SmartDashboard.putNumber("LeftProcessVariable", m_Lencoder.getVelocity());
    SmartDashboard.putNumber("RightProcessVariable", m_Rencoder.getVelocity());


  }
}
