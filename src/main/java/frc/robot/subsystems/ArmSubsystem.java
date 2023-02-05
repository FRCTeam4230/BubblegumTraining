// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorID;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax motor;

  public ArmSubsystem() {
    motor = new CANSparkMax(MotorID.ARM_MOTOR_ID.getId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.arm.ARM_RAMP_RATE);
    //Change to kBreak after testing encoders
    //Leave as kCoast right now so we can move arm around to see if the encoders are recording things right
    motor.setIdleMode(IdleMode.kCoast);
    //Reset encoders
    motor.getEncoder().setPosition(0);
    //Setting conversion factor for encoder
    motor.getEncoder().setPositionConversionFactor(Constants.arm.MOTOR_TO_DEGREES);
    

    SmartDashboard.putData(this);
  }

  public void setAngle(double speed, boolean inverted) {
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
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Arm angle in radians", this::getEncoder, null);
  }
}
