package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final CANSparkMax frontRight =
      new CANSparkMax(DriveTrainConstants.FRONT_RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax frontLeft =
      new CANSparkMax(DriveTrainConstants.FRONT_LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax backRight =
      new CANSparkMax(DriveTrainConstants.BACK_RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax backLeft =
      new CANSparkMax(DriveTrainConstants.BACK_LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
private final DifferentialDrive differentialDrive =
    new DifferentialDrive(frontLeft, frontRight);
  public DrivetrainSubsystem() {
  }
}