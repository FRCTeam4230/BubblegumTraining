// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorID;

import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase {
  private final AHRS navx;

  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  private DifferentialDrive differentialDrive;

  // Dictionary of motor ids with motors
  private Map<MotorID, CANSparkMax> motors = new HashMap<>();
  // Diccionary with motor ids with encoders
  private Map<MotorID, RelativeEncoder> motorEncoders = new HashMap<>();

  // Function for configuring spark maxes
  private static Function<MotorID, CANSparkMax> initiateMotors = (id) -> {// Lambda notation
    CANSparkMax motor = new CANSparkMax(id.getId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.driveTrain.DRIVE_RAMP_RATE);
    motor.setIdleMode(IdleMode.kCoast);

    RelativeEncoder maxEncoder = motor.getEncoder();
    maxEncoder.setPositionConversionFactor(Constants.driveTrain.MOTOR_ROTATION_TO_INCHES);

    return motor;
  };

  public DriveTrain(List<MotorID> motorIds) {
    super();

    navx = new AHRS(SPI.Port.kMXP);
    navx.calibrate();

    motorIds.forEach(motorId -> {
      CANSparkMax controller = initiateMotors.apply(motorId);
      motors.put(motorId, controller);
      motorEncoders.put(motorId, controller.getEncoder());

      switch (motorId) {
        case LEFT_1_MOTOR_ID:
        case LEFT_2_MOTOR_ID:
          controller.setInverted(true);
          break;
        default:
          break;
      }

    });

    // Building motor groups
    leftGroup = new MotorControllerGroup(motors.get(MotorID.LEFT_1_MOTOR_ID),
        motors.get(MotorID.LEFT_2_MOTOR_ID));
    rightGroup = new MotorControllerGroup(motors.get(MotorID.RIGHT_1_MOTOR_ID),
        motors.get(MotorID.RIGHT_2_MOTOR_ID));

    // leftGroup.setInverted(true);

    // Building differential drive
    differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    differentialDrive.setDeadband(0.05); // this is for worn out controllers. dial in if needed

    resetEncoders();
    

    SmartDashboard.putData(this);
    SmartDashboard.putData(navx);
    SmartDashboard.putData(differentialDrive);
  }

  /*
   * arcade drive. speed and rotation
   */
  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed, rotation);
  }

  public void stop() {
    differentialDrive.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void resetEncoders() {
    motorEncoders.values().forEach(encoder -> encoder.setPosition(0));
  }

  private double getLeftEncoder() {
    return (motorEncoders.get(MotorID.LEFT_1_MOTOR_ID).getPosition()
        + motorEncoders.get(MotorID.LEFT_2_MOTOR_ID).getPosition()) / 2;
  }

  private double getRightEncoder() {
    return (motorEncoders.get(MotorID.RIGHT_1_MOTOR_ID).getPosition()
        + motorEncoders.get(MotorID.RIGHT_2_MOTOR_ID).getPosition()) / 2;
  }

  private double getAverageEncoder() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
  }

  public  double getRobotPitch() {
    return navx.getPitch();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    //Gets the remainder of the gyro angle and 360
    //Returns angle between 0 and 360
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return navx.getRate();
  }

  public double getSetPoint() {
    return 1.0;
  }

  public Boolean isLevel(){
    return navx.getPitch() <= getSetPoint()+Constants.driveTrain.kPositionTolerance 
      && navx.getPitch() >= getSetPoint()-Constants.driveTrain.kPositionTolerance;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Left Encoder: ", this::getLeftEncoder, null);
    builder.addDoubleProperty("Right Encoder: ", this::getRightEncoder, null);
    builder.addDoubleProperty("Average Encoder: ", this::getAverageEncoder, null);
    builder.addDoubleProperty("Gyro Pitch", navx::getPitch, null);
    builder.addDoubleProperty("Gyro QuarternionX", navx::getQuaternionX, null);
    builder.addDoubleProperty("Gyro rate: ", navx::getRate, null);
    builder.addDoubleProperty("Gyro get heading: ", this::getHeading, null);
    builder.addDoubleProperty("Gyro get angle: ", navx::getAngle, null);
    builder.addDoubleProperty("getVelocityX: ", navx::getVelocityX, null);
    builder.addDoubleProperty("getVelocityY: ", navx::getVelocityY, null);
    builder.addDoubleProperty("getVelocityZ: ", navx::getVelocityZ, null);
    builder.addBooleanProperty("Level", this::isLevel, null);
    navx.initSendable(builder);
    differentialDrive.initSendable(builder);

  }

}
