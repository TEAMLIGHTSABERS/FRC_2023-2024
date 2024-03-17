package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private double kMaxOutput, kMinOutput;
  private boolean m_launcherRunning;

  private double maxMotorRPM;
  private static double leftCmdWheelRate;
  private static double rightCmdWheelRate; 

  private CANSparkMax leftLaunchWheel;
  private CANSparkMax rightLaunchWheel;

  private SparkPIDController m_leftlancherpidCtrl;
  private RelativeEncoder m_Lencoder;
  public double kLP, kLI, kLD, kLIz, kLFF ;

  private SparkPIDController m_rightlancherpidCtrl;
  private RelativeEncoder m_Rencoder;
  public double kRP, kRI, kRD, kRIz, kRFF ;
  
  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // Set private holding variables: -----------------------------------------------------|
    // launcher status
    m_launcherRunning = false;

    // current running power
    leftCmdWheelRate = 1; // RPM
    rightCmdWheelRate = 1;

    // Set SparkMax motor limits
    kMaxOutput = 1; 
    kMinOutput = -1;

    // The "Unit Max Commanded Velocity" is 1.0. When this commanded
    // velocity as the setpoint velocity of a Feed-Forward Only PID
    // with a Feed-Forward Coefficient of 1.0, the the maximum achieved
    // velocity is 4.3. An arbitrary factor of 1000 is used as the 
    // conversion from "Unit Max Commanded Velocity" to RPM.
    maxMotorRPM = 4300;

    // initialize the left motor PID coefficients
    kLP = 6e-5; 
    kLI = 0;
    kLD = 0; 
    kLIz = 0; 
    kLFF = 0.0004; 

    // initialize the right motor PID coefficients
    kRP = 6e-5; 
    kRI = 0;
    kRD = 0; 
    kRIz = 0; 
    kRFF = 0.0004; 

    // create two new SPARK MAXs and configure them ---------------------------------------|
    // 1 motor for the left launcher wheel: -------------------------------------------||
    leftLaunchWheel =
        new CANSparkMax(Constants.Launcher.kLTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    leftLaunchWheel.setInverted(false);
    leftLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    leftLaunchWheel.setIdleMode(IdleMode.kBrake);
    
    // create a PID controller for the left launcher motor 
    m_leftlancherpidCtrl = leftLaunchWheel.getPIDController();

    // encoder object created to display position and velocity values for the left motor
    m_Lencoder = leftLaunchWheel.getEncoder();

    // configure the left motor PID controller
    m_leftlancherpidCtrl.setP(kLP);
    m_leftlancherpidCtrl.setI(kLI);
    m_leftlancherpidCtrl.setD(kLD);
    m_leftlancherpidCtrl.setIZone(kLIz);
    m_leftlancherpidCtrl.setFF(kLFF);
    m_leftlancherpidCtrl.setOutputRange(kMinOutput, kMaxOutput);

    // Push left motor configuration to the left motor flash memory.
    leftLaunchWheel.burnFlash();

    // 1 motor for the right launcher wheel: -------------------------------------------||
    rightLaunchWheel =
    new CANSparkMax(Constants.Launcher.kRTSCanId, CANSparkLowLevel.MotorType.kBrushless);

    rightLaunchWheel.setInverted(true);
    rightLaunchWheel.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    rightLaunchWheel.setIdleMode(IdleMode.kBrake);

    // create a PID controller for the left launcher motor 
    m_rightlancherpidCtrl = rightLaunchWheel.getPIDController();

    // encoder object created to display position and velocity values for the right motor
    m_Rencoder = rightLaunchWheel.getEncoder();

    // configure the left motor PID controller
    m_rightlancherpidCtrl.setP(kRP);
    m_rightlancherpidCtrl.setI(kRI);
    m_rightlancherpidCtrl.setD(kRD);
    m_rightlancherpidCtrl.setIZone(kRIz);
    m_rightlancherpidCtrl.setFF(kRFF);
    m_rightlancherpidCtrl.setOutputRange(kMinOutput, kMaxOutput);

    // push left motor configuration to the left motor flash memory.
    rightLaunchWheel.burnFlash();
  }

/**
* Constructs a command that starts the launcher and then runs the Intake feeder
* motor to put the Note up to the spinning launcher wheels. After a few more seconds
* the command will shutdown the Launcher Wheels and an the Intake feeder. This
* command takes control of the intake subsystem to make sure the feeder keeps
* running during the launch sequence.
*
* @param _intake The instance of the intake subsystem
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
* Constructs a command that starts the launcher and then runs the Intake feeder
* motor to put the Note up to the spinning launcher wheels. After a few more seconds
* the command will shutdown the Launcher Wheels and an the Intake feeder. This
* command takes control of the intake subsystem to make sure the feeder keeps
* running during the launch sequence.
*
* @return The launch command
*/

public Command testFlyWheels() {
    Command startTest =
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
          }

          @Override
          public boolean isFinished() {
            /* The Launcher and Intake feeder will stop after an
             * appropriate delay.
             */
            return m_timer.get() > Constants.Launcher.kFlyWheelStopTime;
          }

          @Override
          public void end(boolean interrupted) {
            /* Stop both the Launcher and the Intake feeder */
            m_launcherRunning = false;
          }
        };

        return startTest;
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

    double leftSetPoint;
    double rightSetPoint;

    double max = 1.0; //SmartDashboard.getNumber("Max Output", 0);
    double min = -1.0; //SmartDashboard.getNumber("Min Output", 0);
    // read PID coefficients from SmartDashboard
    //    double lp = SmartDashboard.getNumber("P Gain", 0);
    //    double li = SmartDashboard.getNumber("I Gain", 0);
    //    double ld = SmartDashboard.getNumber("D Gain", 0);
    //    double liz = SmartDashboard.getNumber("I Zone", 0);
    //    double lff = SmartDashboard.getNumber("Feed Forward", 0);
    //    double max = SmartDashboard.getNumber("Max Output", 0);
    //    double min = SmartDashboard.getNumber("Min Output", 0);

    double lp = 0; //SmartDashboard.getNumber("P Gain", 0);
    double li = 0; //SmartDashboard.getNumber("I Gain", 0);
    double ld = 0; //SmartDashboard.getNumber("D Gain", 0);
    double liz = 0; //SmartDashboard.getNumber("I Zone", 0);
    double lff = 0.5; //SmartDashboard.getNumber("Feed Forward", 0);
    Shuffleboard.getTab("Launcher Subsystem").add("L P Gain", lp);
//    SmartDashboard.putNumber("L P Gain", lp);
    SmartDashboard.putNumber("L I Gain", li);
    SmartDashboard.putNumber("L D Gain", ld);
    SmartDashboard.putNumber("L I Zone", liz);
    SmartDashboard.putNumber("L Feed Forward", lff);

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
    double rp = 0; //double rp = SmartDashboard.getNumber("P Gain", 0);
    double ri = 0; //double ri = SmartDashboard.getNumber("I Gain", 0);
    double rd = 0; //double rd = SmartDashboard.getNumber("D Gain", 0);
    double riz = 0; //double riz = SmartDashboard.getNumber("I Zone", 0);
    double rff = 0.5;
    //double rff = SmartDashboard.getNumber("R Feed Forward", .2);
//    leftSetPoint = SmartDashboard.getNumber("Left Command Velocity", 0);

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

      leftSetPoint = leftCmdWheelRate; // leftCmdWheelRate/maxMotorRPM;
      rightSetPoint = rightCmdWheelRate; // rightCmdWheelRate/maxMotorRPM;
//      rightLaunchWheel.set(m_rightlancherpidCtrl.output());
//      leftLaunchWheel.set(m_leftlancherpidCtrl.getOutputMax());

//      rightLaunchWheel.set(Constants.Launcher.kRightPower);
//      leftLaunchWheel.set(Constants.Launcher.kLeftPower);

    } else {
      leftSetPoint = 0;
      rightSetPoint = 0;

//      rightLaunchWheel.set(0.0);
//      leftLaunchWheel.set(0.0);
    }

    m_leftlancherpidCtrl.setReference(leftSetPoint, CANSparkMax.ControlType.kVelocity);
    m_rightlancherpidCtrl.setReference(rightSetPoint, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putBoolean("Launcher Running", m_launcherRunning);
    SmartDashboard.putNumber("Left Command Velocity", leftSetPoint);
//    SmartDashboard.putNumber("RightSetPoint", rightSetPoint);
    SmartDashboard.putNumber("Left Velocity", m_Lencoder.getVelocity());
//    SmartDashboard.putNumber("Right Velocity", m_Rencoder.getVelocity());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("leftWheelRate", LauncherSubsystem::getLeftWheelRate, null);
    builder.addDoubleProperty("RightWheelRate", LauncherSubsystem::getRightWheelRate, null);
  }

  static public double getLeftWheelRate(){
    return leftCmdWheelRate;
  }

  public static double getRightWheelRate(){
    return rightCmdWheelRate;
  }
}
