package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // Class variables

    private static int selectedPosition;
    private static double cmdedElevMotorPos;
    private static double currElevMotorPos;
    private SparkPIDController elevationPIDCtrl;
    private static int inputDelayCtr;

    private static double kStartDeg, kHighShotDeg, kAmpDeg;
    private static double startDeg, highShotDeg, ampDeg;
    private static double kHighShotFFGainUp;
    private static double kHighShotPGainUp;

    // Class Hardware
    private CANSparkMax elevationMotor;
    private RelativeEncoder elevationRelEncoder;
    private Servo deflServo;

    // Class Constructor
    public TurretSubsystem(){
        // Tunable Constants
        kStartDeg = Constants.Turret.kStartPosition;
        kHighShotDeg = Constants.Turret.kHighShotPosition; //Speaker
        kAmpDeg = Constants.Turret.kAmpPosition;
        kHighShotFFGainUp = Constants.Turret.kVEGainStartUp;
        kHighShotPGainUp = Constants.Turret.kPEGainStartUp;

        // Initialize operational variables
        selectedPosition = 0; // Hanging Position
        currElevMotorPos = convertSelPosToWench(selectedPosition);
        cmdedElevMotorPos = currElevMotorPos;
        inputDelayCtr = 0;
    
        // Initialize the Turret's SmartDashboard parameters to their default constants
        SmartDashboard.putNumber("Sel 0 Pos", kStartDeg);
        SmartDashboard.putNumber("Sel 2 Pos", kHighShotDeg);
        SmartDashboard.putNumber("Sel 3 Pos", kAmpDeg);
        SmartDashboard.putNumber("High Shot FF Gain Up", kHighShotFFGainUp);
        SmartDashboard.putNumber("High Shot P Gain Up", kHighShotPGainUp);

        SmartDashboard.putNumber("Curr Sel Pos", selectedPosition);
        SmartDashboard.putNumber("Curr Elev Motor Pos", currElevMotorPos);
        SmartDashboard.putNumber("Cmded Elev Motor Pos", cmdedElevMotorPos);

        SmartDashboard.putData("Accept 0th", acceptZeroSetting());
        SmartDashboard.putData("Accept 2nd", acceptTwoSetting());
        SmartDashboard.putData("Accept 3rd", acceptThreeSetting());
        SmartDashboard.putData("Accept FF Gain High Shot Up", acceptFFGainSetting());
        SmartDashboard.putData("Accept P Gain High Shot Up", acceptPGainSetting());
    
        // Initialize Hardware
        elevationMotor =
        new CANSparkMax(Constants.Turret.kTurCanId, CANSparkLowLevel.MotorType.kBrushless);

        elevationMotor.setInverted(false );
        elevationMotor.setSmartCurrentLimit((int) kHighShotFFGainUp);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        elevationRelEncoder = elevationMotor.getEncoder();
        elevationRelEncoder.setPositionConversionFactor(22.0); // Degrees/Shaft
        elevationRelEncoder.setVelocityConversionFactor(22.0); // dpm/Shaft
        elevationRelEncoder.setPosition(0.0);


        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(kHighShotPGainUp);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFF(kHighShotFFGainUp);
        elevationPIDCtrl.setFeedbackDevice(elevationRelEncoder);
        elevationPIDCtrl.setOutputRange(-1, 1); 

        deflServo = new Servo(Constants.Turret.kDeflID);
        deflServo.set(Constants.Turret.kDeflOff);

    }

    /**
    * Method to drive the turret elevation to one of three positions.
    *
    * @param povDegrees  The position indicator from the POV.
    *                         Up is Speaker Position,
    *                         Right is Amp position, and
    *                         Down is Center Stand Positon.
    */
    public void driveWench(Boolean upCommand, Boolean downCommand) {

        if(upCommand){
            advancePOS();
        } else if (downCommand){
            if (inputDelayCtr == Constants.OIConstants.kInputDelayTimedOut){
                reducePOS();
                inputDelayCtr = 0;
            }
        }
    };

    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the Zeroth Turret Position.
    *
    * @return The setStart command
    */
    public Command acceptZeroSetting(){
    Command setStart = 
        new Command() {
        @Override
        public void initialize() {
            startDeg = SmartDashboard.getNumber("Sel 0 Pos", kStartDeg);
            kStartDeg = startDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setStart;
    };

    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the 2nd Turret Position.
    *
    * @return The SetHighShot command
    */
    public Command acceptTwoSetting(){
    Command setHighShot = 
        new Command() {
        @Override
        public void initialize() {
            highShotDeg = SmartDashboard.getNumber("Sel 2 Pos", kHighShotDeg);
            kHighShotDeg = highShotDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setHighShot;
    };

    /**
    * Constructs a command for a button that accepts the Gear position (in deg) 
    * for the 3rd Turret Position.
    *
    * @return The SetAmp command
    */
    public Command acceptThreeSetting(){
    Command setAmp = 
        new Command() {
        @Override
        public void initialize() {
            ampDeg = SmartDashboard.getNumber("Sel 3 Pos", kAmpDeg);
            kAmpDeg = ampDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setAmp;
    };

    /**
    * Constructs a command for a button that accepts the Current Limit (percentage).
    *
    * @return The SetCurrLmt command
    */
    public Command acceptFFGainSetting(){
    Command setCurrLmt = 
        new Command() {
        @Override
        public void initialize() {
            double highShotFFGainUp = SmartDashboard.getNumber("High Shot FF Gain Up", kHighShotFFGainUp);
            kHighShotFFGainUp = highShotFFGainUp;
            elevationPIDCtrl.setFF(kHighShotFFGainUp);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setCurrLmt;
    };
    
    /**
    * Constructs a command for a button that accepts the P Gain.
    *
    * @return The SetPGain command
    */
    public Command acceptPGainSetting(){
    Command setCurrLmt = 
        new Command() {
        @Override
        public void initialize() {
            double highShotPGainUp = SmartDashboard.getNumber("High Shot P Gain Up", kHighShotPGainUp);
            kHighShotPGainUp = highShotPGainUp;
            elevationPIDCtrl.setP(kHighShotPGainUp);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setCurrLmt;
    };

    @Override
    public void periodic() { // this method will be called once per scheduler run
        // set the launcher motor powers based on whether the launcher is on or not
        double posError;

        if (inputDelayCtr < Constants.OIConstants.kInputDelayTimedOut){
            inputDelayCtr++;
        }

        cmdedElevMotorPos = convertSelPosToWench(selectedPosition);
        currElevMotorPos = elevationRelEncoder.getPosition();

        posError = cmdedElevMotorPos - currElevMotorPos;
        elevationPIDCtrl.setReference(posError, ControlType.kPosition);

        // Add Launcher Power Wheel Rates to the Launcher Subsystem Tab on Shuffleboard.
        SmartDashboard.putNumber("Curr Sel Pos", selectedPosition);
        SmartDashboard.putNumber("Curr Elev Motor Pos", currElevMotorPos);
        SmartDashboard.putNumber("Cmded Elev Motor Pos", cmdedElevMotorPos);

        SmartDashboard.putNumber("Commanded Wench Position", cmdedElevMotorPos);

    }

    public int getSelPosition (){
        return (selectedPosition);
    }

    private double convertSelPosToWench (int _SelectedPos) {
        double wenchPosition;

        switch (_SelectedPos) {
        case 0: // Start Position
            wenchPosition = kStartDeg;    // Rotations
            break;
        /*case 1: // Speaker Position
            wenchPosition = kLongShotDeg;   // Rotations
            break;*/
        case 2: // Speaker Position
            wenchPosition = kHighShotDeg;   // Rotations
            break;
        case 3: // Amp Position
        default:
            wenchPosition = kAmpDeg;  // Rotations
            break;
        };

        return wenchPosition;
    }

    public void advancePOS(){
        if (selectedPosition < Constants.Turret.kAmpID){
            selectedPosition++;
            if(selectedPosition == 1){
                selectedPosition++;
            }

            switch (selectedPosition) {
                case Constants.Turret.kHighShotID:

                    double highShotPGainUp = SmartDashboard.getNumber("High Shot P Gain Up", kHighShotPGainUp);
                    double highShotFFGainUp = SmartDashboard.getNumber("High Shot FF Gain Up", kHighShotFFGainUp);

                    kHighShotPGainUp = highShotPGainUp;
                    kHighShotFFGainUp = highShotFFGainUp; 
                    elevationPIDCtrl.setP(kHighShotPGainUp);
                    elevationPIDCtrl.setFF(kHighShotFFGainUp);

                    break;
            
                case Constants.Turret.kAmpID:

                    deflServo.set(Constants.Turret.kDeflON);

                    kHighShotPGainUp = Constants.Turret.kPEGainAmpUp;
                    kHighShotFFGainUp = Constants.Turret.kVEGainAmpUp; 
                    elevationPIDCtrl.setP(kHighShotPGainUp);
                    elevationPIDCtrl.setFF(kHighShotFFGainUp);

                    break;
            
                default:

                    // Do Nothing

                    break;
            }

            SmartDashboard.putNumber("Curr Sel Pos", selectedPosition);
        }
    }

    public void reducePOS(){
        if (selectedPosition > Constants.Turret.kStartID){
            selectedPosition--;
            if(selectedPosition == 1){
                selectedPosition--;
            }

            switch (selectedPosition) {
                case Constants.Turret.kHighShotID:

                    deflServo.set(Constants.Turret.kDeflOff);

                    kHighShotPGainUp = Constants.Turret.kPEGainHighShotDown;
                    kHighShotFFGainUp = Constants.Turret.kVEGainHighShotDown; 
                    elevationPIDCtrl.setP(kHighShotPGainUp);
                    elevationPIDCtrl.setFF(kHighShotFFGainUp);

                    break;
            
                case Constants.Turret.kStartID:
                    kHighShotPGainUp = Constants.Turret.kPEGainStartDown;
                    kHighShotFFGainUp = Constants.Turret.kVEGainStartDown; 
                    elevationPIDCtrl.setP(kHighShotPGainUp);
                    elevationPIDCtrl.setFF(kHighShotFFGainUp);

                    break;
            
                default:
                    
                    // Do Nothing

                    break;
            }

            SmartDashboard.putNumber("Curr Sel Pos", selectedPosition); 
        }
    }
}
