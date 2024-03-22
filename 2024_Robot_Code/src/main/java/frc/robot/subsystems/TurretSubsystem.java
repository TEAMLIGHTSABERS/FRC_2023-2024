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
        elevationAbsEncoder.setZeroOffset(currentWenchPosition);

        elevationPIDCtrl = elevationMotor.getPIDController();
        elevationPIDCtrl.setP(Constants.Turret.kPEController);
        elevationPIDCtrl.setI(Constants.Turret.kIEController);
        elevationPIDCtrl.setD(Constants.Turret.kDEController);
        elevationPIDCtrl.setFeedbackDevice(elevationAbsEncoder);
    }

    /**
    * Method to drive the turret elevation to one of three positions.
    *
    * @param povDegrees  The position indicator from the POV.
    *                         Up is Speaker Position,
    *                         Right is Amp position, and
    *                         Down is Center Stand Positon.
    */
    public void driveWench(double _povPos) {

        if ((_povPos > 315) && (_povPos <= 45)){
            SelectedPosition = 0;
        } else if ((_povPos > 45) && (_povPos <= 135)){
            SelectedPosition = 1;
        } else if ((_povPos > 135) && (_povPos <= 225)){
            SelectedPosition = 2;
        } else {
            // SelectedPosition does not change
        }
            
        commandedWenchPosition = getSelPos(SelectedPosition);
  
        elevationPIDCtrl.setReference(commandedWenchPosition, ControlType.kPosition);

        currentWenchPosition = elevationAbsEncoder.getPosition();
        SmartDashboard.putNumber("Commanded Wench Position", commandedWenchPosition);
        SmartDashboard.putNumber("Current Wench Position", currentWenchPosition);
    };

    private double getSelPos (int _SelectedPos) {
      double wenchPosition;

      switch (_SelectedPos) {
        case 0: // Speaker Position
            wenchPosition = 1000.0;
            break;
        case 1: // Amplifier Position
            wenchPosition = 500.0;
            break;
        case 2: // Hanging Position
        default:
            wenchPosition = 0.0;
            break;
      };

      return wenchPosition;
    }
}