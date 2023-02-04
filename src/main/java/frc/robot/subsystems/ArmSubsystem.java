// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax motor;

  public ArmSubsystem() {
    motor = new CANSparkMax(Constants.arm.ARM_MOTOR_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.arm.ARM_RAMP_RATE);
    //Reset encoders
    motor.getEncoder().setPosition(0);
  }

  public void setSpeed(double speed, boolean inverted) {
    motor.setInverted(inverted);
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public double getEncoder() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
