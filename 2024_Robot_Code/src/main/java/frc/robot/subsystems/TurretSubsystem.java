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
    private GenericEntry currSelPosEntry;
    private GenericEntry currWenchPosEntry;

    private static int selectedPosition;
    private static double commandedWenchPosition;
    private static double currentWenchPosition;
    private SparkPIDController elevationPIDCtrl;
    private static int inputDelayCtr;

    private static double kStartDeg, kSpeakerDeg, kAmpDeg;
    private static double startDeg, speakerDeg, ampDeg;

    // Class Hardware
    private CANSparkMax elevationMotor;
    private RelativeEncoder elevationRelEncoder;

    // Class Constructor
    public TurretSubsystem(){
        selectedPosition = 0; // Hanging Position
        currentWenchPosition = convertSelPosToWench(selectedPosition);
        commandedWenchPosition = currentWenchPosition;
        inputDelayCtr = 0;

        kStartDeg = Constants.Turret.kStartPosition;
        kSpeakerDeg = Constants.Turret.kSpeakerPosition;
        kAmpDeg = Constants.Turret.kAmpPosition;

        // Turret status
        TurretTab = Shuffleboard.getTab("Turret Subsystem");
        selectZeroEntry = TurretTab
        .add("Select 0 Position", kStartDeg).getEntry();
        selectOneEntry = TurretTab
        .add("Select 1 Position", kSpeakerDeg).getEntry();
        selectTwoEntry = TurretTab
        .add("Select 2 Position", kAmpDeg).getEntry();
        currSelPosEntry = TurretTab
        .add("Current Selected Position", kStartDeg).getEntry();
        currWenchPosEntry = TurretTab
        .add("Current Turrent Position", currentWenchPosition).getEntry();

        TurretTab.add("Accept 0th Pos", acceptZeroSetting());
        TurretTab.add("Accept 1st Pos", acceptOneSetting());
        TurretTab.add("Accept 2nd Pos", acceptTwoSetting());
    
        elevationMotor =
        new CANSparkMax(Constants.Turret.kTurCanId, CANSparkLowLevel.MotorType.kBrushless);

        elevationMotor.setInverted(false );
        elevationMotor.setSmartCurrentLimit(Constants.Turret.kTurCurrentLimit);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        elevationRelEncoder = elevationMotor.getEncoder();
        elevationRelEncoder.setPositionConversionFactor(22.0); // Degrees/Shaft
        elevationRelEncoder.setVelocityConversionFactor(22.0); // dpm/Shaft
        elevationRelEncoder.setPosition(currentWenchPosition);


        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(Constants.Turret.kPEController);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFF(Constants.Turret.kVEController);
        elevationPIDCtrl.setFeedbackDevice(elevationRelEncoder);
        elevationPIDCtrl.setOutputRange(-0.2, 1); 

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
    * @return The SetAmp command
    */
    public Command acceptTwoSetting(){
    Command setAmp = 
        new Command() {
        @Override
        public void initialize() {
            ampDeg = selectTwoEntry.getDouble(kAmpDeg); 
            kAmpDeg = ampDeg;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
        };

    return setAmp;
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
        case 2: // Amp Position
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
            currSelPosEntry.setInteger(selectedPosition);
            SmartDashboard.putNumber("Selected Position", selectedPosition);
        }
    }
}
