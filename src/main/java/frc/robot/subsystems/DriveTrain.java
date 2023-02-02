// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  private final MotorControllerGroup leftGroup;
  private final MotorControllerGroup rightGroup;

  private CANSparkMax left1Motor;
  private CANSparkMax left2Motor;
  private CANSparkMax right1Motor;
  private CANSparkMax right2Motor;

  private DifferentialDrive differentialDrive;

  public DriveTrain() {
    //Building motor controllers
    left1Motor = new CANSparkMax(Constants.driveTrain.LEFT_1_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    left2Motor = new CANSparkMax(Constants.driveTrain.LEFT_2_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    right1Motor = new CANSparkMax(Constants.driveTrain.RIGHT_1_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    right2Motor = new CANSparkMax(Constants.driveTrain.RIGHT_2_MOTOR_ID, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);

    //Configuring motor controllers
    initiateMotors(left1Motor);
    initiateMotors(left2Motor);
    initiateMotors(right1Motor);
    initiateMotors(right2Motor);

    
    //Building motor groups
    leftGroup = new MotorControllerGroup(left1Motor, left2Motor);
    rightGroup = new MotorControllerGroup(right1Motor, right2Motor);

    leftGroup.setInverted(true);

    //Building differential drive
    differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    differentialDrive.setDeadband(0.05);
  }

  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed * Constants.driveTrain.SPEED_MULTIPLIER, 
    rotation * Constants.driveTrain.ROTATION_MULTIPLIER);
  }

  public void stop() {
    differentialDrive.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initiateMotors(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.driveTrain.DRIVE_RAMP_RATE);
  }
}
