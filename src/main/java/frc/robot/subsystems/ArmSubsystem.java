// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorID;

public class ArmSubsystem extends SubsystemBase {
  private final DutyCycleEncoder encoder;
  private final CANSparkMax motor;
  private final DigitalInput frontLimit;
  private final DigitalInput backLimit;
  

  public ArmSubsystem() {
    motor = new CANSparkMax(MotorID.ARM_MOTOR_ID.getId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(Constants.arm.ARM_RAMP_RATE);
    //Change to kBreak after testing encoders
    //Leave as kCoast right now so we can move arm around to see if the encoders are recording things right
    motor.setIdleMode(IdleMode.kCoast);
    //Setting conversion factor for encoder
    motor.getEncoder().setPositionConversionFactor(Constants.arm.MOTOR_TO_DEGREES);

    encoder = new DutyCycleEncoder(Constants.arm.ENCODER_PORT);
    frontLimit = new DigitalInput(Constants.arm.FRONT_LIMIT_PORT);
    backLimit = new DigitalInput(Constants.arm.BACK_LIMIT_PORT);

    resetEncoders();
    

    SmartDashboard.putData(this);
    SmartDashboard.putData(encoder);
    SmartDashboard.putData(frontLimit);
    SmartDashboard.putData(backLimit);

  }

  public void setAngle(double speed, boolean forward) {
    
    motor.setInverted(forward);
    motor.set(MathUtil.clamp(speed, -0.99, 0.99));
  }

  public void stop() {
    motor.set(0);
  }

  public double getMotorEncoderPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //   if ((isForward() || (isBack() {
  //     stop();
  //   }

  //   if (isDown()) {
  //     resetEncoder();
  //   }
  }

  public boolean isForward() {
    return !frontLimit.get() || getPosition() >= Constants.arm.FORWARD_LIMIT_ANGLE;
  }

  public boolean isBack() {
    return !backLimit.get() || getPosition() <= Constants.arm.BACK_LIMIT_ANGLE;
  }

  public double getPosition() {
    //Check which value to get from encoder
    return encoder.get();
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Arm encoder: ", this::getMotorEncoderPosition, null);
    encoder.initSendable(builder);
    frontLimit.initSendable(builder);
    backLimit.initSendable(builder);

  }

  private void resetEncoders() {
    encoder.reset();
    motor.getEncoder().setPosition(0);
  
  }
}
