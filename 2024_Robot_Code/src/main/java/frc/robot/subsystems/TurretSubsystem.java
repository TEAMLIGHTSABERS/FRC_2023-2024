package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // Class variables
    private ShuffleboardTab TurretTab;
    private GenericEntry selectZeroEntry;
    private GenericEntry selectOneEntry;
    private GenericEntry selectTwoEntry;
    private GenericEntry selectThreeEntry;
    private GenericEntry selectFFGainEntry;
    private GenericEntry currSelPosEntry;
    private GenericEntry currWenchPosEntry;

    private static int selectedPosition;
    private static double commandedWenchPosition;
    private static double currentWenchPosition;
    private SparkPIDController elevationPIDCtrl;
    private static int inputDelayCtr;
    private static Boolean resetEncoder;

    private static double kStartDeg, kSpeakerDeg, kHighShotDeg, kAmpDeg;
    private static double startDeg, speakerDeg, highShotDeg, ampDeg;
    private static double kFFGain, fFGain;

    // Class Hardware
    private CANSparkMax elevationMotor;
    private RelativeEncoder elevationRelEncoder;

    // Class Constructor
    public TurretSubsystem(){
        // Tunable Constants
        kStartDeg = Constants.Turret.kStartPosition;
        kSpeakerDeg = Constants.Turret.kSpeakerPosition;
        kHighShotDeg = Constants.Turret.kHighShotPosition;
        kAmpDeg = Constants.Turret.kAmpPosition;
        kFFGain = Constants.Turret.kVEController;

        // Initialize operational variables
        selectedPosition = 0; // Hanging Position
        currentWenchPosition = convertSelPosToWench(selectedPosition);
        commandedWenchPosition = currentWenchPosition;
        inputDelayCtr = 0;
        resetEncoder = false;

        // Turret status
        TurretTab = Shuffleboard.getTab("Turret Subsystem");

        selectZeroEntry = TurretTab
        .add("Sel 0 Pos", kStartDeg).getEntry();
        selectOneEntry = TurretTab
        .add("Sel 1 Pos", kSpeakerDeg).getEntry();
        selectTwoEntry = TurretTab
        .add("Sel 2 Pos", kHighShotDeg).getEntry();
        selectThreeEntry = TurretTab
        .add("Sel 3 Pos", kAmpDeg).getEntry();
        selectFFGainEntry = TurretTab
        .add("Sel FF Gain", kFFGain).getEntry();

        currSelPosEntry = TurretTab
        .add("Current Selected Position", kStartDeg).getEntry();
        currWenchPosEntry = TurretTab
        .add("Current Turrent Position", currentWenchPosition).getEntry();

        TurretTab.add("Accept 0th", acceptZeroSetting());
        TurretTab.add("Accept 1st", acceptOneSetting());
        TurretTab.add("Accept 2nd", acceptTwoSetting());
        TurretTab.add("Accept 3rd", acceptThreeSetting());
        TurretTab.add("Accept FFGain", acceptFFGainSetting());
    
        elevationMotor =
        new CANSparkMax(Constants.Turret.kTurCanId, CANSparkLowLevel.MotorType.kBrushless);

        elevationMotor.setInverted(false );
        elevationMotor.setSmartCurrentLimit((int) kFFGain);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        elevationRelEncoder = elevationMotor.getEncoder();
        elevationRelEncoder.setPositionConversionFactor(22.0); // Degrees/Shaft
        elevationRelEncoder.setVelocityConversionFactor(22.0); // dpm/Shaft
        elevationRelEncoder.setPosition(0.0);


        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(Constants.Turret.kPEController);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFF(Constants.Turret.kVEController);
        elevationPIDCtrl.setFeedbackDevice(elevationRelEncoder);
        elevationPIDCtrl.setOutputRange(-1, 1); 

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
            startDeg = selectZeroEntry.getDouble(kStartDeg); 
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
    * for the 1st Turret Position.
    *
    * @return The setSpeaker command
    */
    public Command acceptOneSetting(){
    Command setSpeaker = 
        new Command() {
        @Override
        public void initialize() {
            speakerDeg = selectOneEntry.getDouble(kSpeakerDeg); 
            kSpeakerDeg = speakerDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setSpeaker;
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
            highShotDeg = selectTwoEntry.getDouble(kHighShotDeg); 
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
            ampDeg = selectThreeEntry.getDouble(kAmpDeg); 
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
            fFGain = selectFFGainEntry.getDouble(kFFGain); 
            kFFGain = fFGain;
            elevationPIDCtrl.setFF(kFFGain);
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

        commandedWenchPosition = convertSelPosToWench(selectedPosition);
        currentWenchPosition = elevationRelEncoder.getPosition();

    //        if (currentWenchPosition < commandedWenchPosition){
    //            elevationMotor.set(.5);
    //        } else if (currentWenchPosition > commandedWenchPosition){
    //            elevationMotor.set(-.5);
    //        };
        posError = commandedWenchPosition - currentWenchPosition;
        elevationPIDCtrl.setReference(posError, ControlType.kPosition);

        //if ((resetEncoder == true) && (currentWenchPosition < kStartDeg + 5)) {
        //    elevationPIDCtrl.setFF(0.0);
        //}

        // Add Launcher Power Wheel Rates to the Launcher Subsystem Tab on Shuffleboard.
        currSelPosEntry.setInteger(selectedPosition);
        currWenchPosEntry.setDouble(currentWenchPosition);
        SmartDashboard.putNumber("Commanded Wench Position", commandedWenchPosition);
        SmartDashboard.putNumber("Current Wench Position", currentWenchPosition);

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
        case 1: // Speaker Position
            wenchPosition = kSpeakerDeg;   // Rotations
            break;
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
            currSelPosEntry.setInteger(selectedPosition);
            SmartDashboard.putNumber("Selected Position", selectedPosition);
        }
    }

    public void reducePOS(){
        if (selectedPosition > Constants.Turret.kStartID){
            selectedPosition--;
            if (selectedPosition == 0) {
                resetEncoder = true;
            }
            currSelPosEntry.setInteger(selectedPosition);
            SmartDashboard.putNumber("Selected Position", selectedPosition);
        }
    }
}
