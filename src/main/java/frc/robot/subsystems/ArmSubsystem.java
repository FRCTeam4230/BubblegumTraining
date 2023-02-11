// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StaticFunctions;
import frc.robot.Constants.MotorID;

public class ArmSubsystem extends SubsystemBase {
  private final DutyCycleEncoder encoder;
  private final CANSparkMax motor;
  private final DigitalInput frontLimit;
  private final DigitalInput backLimit;
  private boolean goingForward;
  

  public ArmSubsystem() {
    motor = StaticFunctions.initiateCANSparkMaxMotor.apply(MotorID.ARM_MOTOR_ID);


    encoder = new DutyCycleEncoder(Constants.arm.ENCODER_PORT);
    frontLimit = new DigitalInput(Constants.arm.FRONT_LIMIT_PORT);
    backLimit = new DigitalInput(Constants.arm.BACK_LIMIT_PORT);

    resetEncoders();

    encoder.setDistancePerRotation(360);
    
    goingForward = false;

    SmartDashboard.putData(this);
    SmartDashboard.putData(encoder);
    SmartDashboard.putData(frontLimit);
    SmartDashboard.putData(backLimit);

  }


  public void goForward(double speed){
    if (!isForward()){
      motor.setInverted(false);
      motor.set(MathUtil.clamp(speed,-0.99,.99));
    }else{
      stop();
    }
  }

  public void goBackwards(double speed){
    if (!isBack()){
      motor.setInverted(true);
      motor.set(MathUtil.clamp(speed,-.99,.99));
    }else{
      stop();
    }
  }


  public void stop() {
    motor.set(0);
  }

  public double getMotorEncoderPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
  }

  public boolean isForward() {
    return !frontLimit.get();// || getPosition() >= Constants.arm.FORWARD_LIMIT_ANGLE;
  }

  public boolean isBack() {
    return !backLimit.get();// || getPosition() <= Constants.arm.BACK_LIMIT_ANGLE;
  }

  public double getPosition() {
    //Check which value to get from encoder
    return encoder.get();
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);


    builder.addBooleanProperty("GoingForward", () -> goingForward, null);
    builder.addBooleanProperty("IsForward", this::isForward, null);
    builder.addBooleanProperty("isBack", this::isBack, null);
    builder.addDoubleProperty("Arm motor encoder: ", this::getMotorEncoderPosition, null);
    encoder.initSendable(builder);
    frontLimit.initSendable(builder);
    backLimit.initSendable(builder);

  }

  private void resetEncoders() {
    encoder.reset();
    motor.getEncoder().setPosition(0);
  
  }
}
