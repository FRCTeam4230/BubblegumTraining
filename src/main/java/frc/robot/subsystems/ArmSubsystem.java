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

  public void setAngle(double speed, boolean forward) {
    goingForward = forward;
    motor.setInverted(forward);
    //If the arm is going forward and is at the very front or if the arm is going backwards and is at the very back, stop
    //else, move
    if((forward && isForward()) || (!forward && isBack())) {
      stop();
    } else {
      //If none of the previous conditions were satisfied, the arm can move
      motor.set(MathUtil.clamp(speed, -0.99, 0.99));
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
    // This method will be called once per scheduler run
    //If the arm is at the very front and going forward, or if the arm
    //is at the very back and going backwards, stop the arm
    if ((isForward() && goingForward) || (isBack() && !goingForward)) {
      stop();
    }

    if (isForward()) {
      encoder.reset();
    }
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
