package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // Class variables
    private static int SelectedPosition;
    private static double commandedWenchPosition;
    private static double currentWenchPosition;
    private SparkPIDController elevationPIDCtrl;
    private static int inputDelayCtr;

    // Class Hardware
    private CANSparkMax elevationMotor;
    private RelativeEncoder elevationRelEncoder;

    // Class Constructor
    public TurretSubsystem(){
        SelectedPosition = 0; // Hanging Position
        currentWenchPosition = convertSelPosToWench(SelectedPosition);
        commandedWenchPosition = currentWenchPosition;
        inputDelayCtr = 0;

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
            if (inputDelayCtr == Constants.OIConstants.kInputDelayTimedOut){
                advancePOS();
                inputDelayCtr = 0;
            }
        } else if (downCommand){
            reducePOS();
        }
    };

    @Override
    public void periodic() { // this method will be called once per scheduler run
      // set the launcher motor powers based on whether the launcher is on or not
        double posError;

        if (inputDelayCtr < Constants.OIConstants.kInputDelayTimedOut){
            inputDelayCtr++;
        }

        commandedWenchPosition = convertSelPosToWench(SelectedPosition);
        currentWenchPosition = elevationRelEncoder.getPosition();

//        if (currentWenchPosition < commandedWenchPosition){
//            elevationMotor.set(.5);
//        } else if (currentWenchPosition > commandedWenchPosition){
//            elevationMotor.set(-.5);
//        };
        posError = commandedWenchPosition - currentWenchPosition;
        elevationPIDCtrl.setReference(posError, ControlType.kPosition);

        SmartDashboard.putNumber("Commanded Wench Position", commandedWenchPosition);
        SmartDashboard.putNumber("Current Wench Position", currentWenchPosition);

    }

    public int getSelPosition (){
        return (SelectedPosition);
    }

    private double convertSelPosToWench (int _SelectedPos) {
      double wenchPosition;

      switch (_SelectedPos) {
        case 0: // Start Position
            wenchPosition = Constants.Turret.kStartPosition;    // Rotations
            break;
        case 1: // Speaker Position
            wenchPosition = Constants.Turret.kSpeakerPosition;   // Rotations
            break;
        case 2: // Amp Position
        default:
            wenchPosition = Constants.Turret.kAmpPosition;  // Rotations
            break;
      };

      return wenchPosition;
    }

    public void advancePOS(){
        if (SelectedPosition < 2){
            SelectedPosition++;
            SmartDashboard.putNumber("Selected Position", SelectedPosition);
        }
    }

    public void reducePOS(){
        if (SelectedPosition > 0){
            SelectedPosition--;
            SmartDashboard.putNumber("Selected Position", SelectedPosition);
        }
    }

}
