package frc.robot.Subsystem;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax backLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveTrainConstants.FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax backRightMotor = new CANSparkMax(Constants.DriveTrainConstants.BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public DriveTrainSubsystem() {
    }
}

