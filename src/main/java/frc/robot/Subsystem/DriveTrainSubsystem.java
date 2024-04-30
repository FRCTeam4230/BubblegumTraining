package frc.robot.Subsystem;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    // Variables
    private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax backLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveTrainConstants.FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax backRightMotor = new CANSparkMax(Constants.DriveTrainConstants.BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    // Methods
    public DriveTrainSubsystem() {
        configureMotor(frontLeftMotor);
        configureMotor(backLeftMotor);
        configureMotor(frontRightMotor);
        configureMotor(backRightMotor);
        frontRightMotor.setInverted(true);
        backLeftMotor.follow(frontLeftMotor);
        backRightMotor.follow(frontRightMotor);
    }
    public void resetGyroscope() {
        navx.reset();
    }
    private void configureMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setOpenLoopRampRate(Constants.DriveTrainConstants.OPEN_LOOP_RAMP_RATE);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
    public void arcadeDrive(double forward, double turn) {
        differentialDrive.arcadeDrive(forward, turn);
    }
    public void stop() {
        differentialDrive.arcadeDrive(0,0);
    }
    public double getHeading() {
        return navx.getCompassHeading();
    }

}

