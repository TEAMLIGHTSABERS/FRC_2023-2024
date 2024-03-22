package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
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

    // Class Hardware
    private CANSparkMax elevationMotor;
    private AbsoluteEncoder elevationAbsEncoder;

    // Class Constructor
    public TurretSubsystem(){
        SelectedPosition = 0; // Hanging Position
        currentWenchPosition = getSelPos(SelectedPosition);
        commandedWenchPosition = currentWenchPosition;

        elevationMotor =
        new CANSparkMax(Constants.Turret.kTurCanId, CANSparkLowLevel.MotorType.kBrushless);

        elevationMotor.setInverted(false);
        elevationMotor.setSmartCurrentLimit(Constants.Turret.kTurCurrentLimit);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        elevationAbsEncoder = elevationMotor.getAbsoluteEncoder();
        elevationAbsEncoder.setPositionConversionFactor(1.0); // rotations
        elevationAbsEncoder.setVelocityConversionFactor(1.0); // rpm
        elevationAbsEncoder.setZeroOffset(currentWenchPosition);
    

        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(Constants.Turret.kPEController);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFF(Constants.Turret.kVEController);
        elevationPIDCtrl.setFeedbackDevice(elevationAbsEncoder);
        elevationPIDCtrl.setOutputRange(0, 1); 

    }

    /**
    * Method to drive the turret elevation to one of three positions.
    *
    * @param povDegrees  The position indicator from the POV.
    *                         Up is Speaker Position,
    *                         Right is Amp position, and
    *                         Down is Center Stand Positon.
    */
    public void driveWench() {

            
        commandedWenchPosition = getSelPos(SelectedPosition);
  
//        elevationMotor.set(.1);
        elevationPIDCtrl.setReference(commandedWenchPosition, ControlType.kPosition);

        currentWenchPosition = elevationAbsEncoder.getPosition();
        SmartDashboard.putNumber("Commanded Wench Position", commandedWenchPosition);
        SmartDashboard.putNumber("Current Wench Position", currentWenchPosition);
    };

    private double getSelPos (int _SelectedPos) {
      double wenchPosition;

      switch (_SelectedPos) {
        case 0: // Speaker Position
            wenchPosition = Constants.Turret.kSpeakerPosition;  // Rotations
            break;
        case 1: // Amplifier Position
            wenchPosition = Constants.Turret.kAmpPosition;   // Rotations
            break;
        case 2: // Hanging Position
        default:
            wenchPosition = Constants.Turret.kHangingPosition;    // Rotations
            break;
      };

      return wenchPosition;
    }

    public void advancePOS(){
        if (SelectedPosition < 3){
            SelectedPosition++;
        }
    }

    public void reducePOS(){
        if (SelectedPosition > -1){
            SelectedPosition--;
        }
    }

}
