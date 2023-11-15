// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

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
  private final DutyCycleEncoder rotaryEncoder;
  private final CANSparkMax motor;
  private final DigitalInput frontLimit;
  private final DigitalInput backLimit;
  private boolean goingForward;

  public ArmSubsystem() {
    motor = StaticFunctions.initiateCANSparkMaxMotor.apply(MotorID.ARM_MOTOR_ID);

    rotaryEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    frontLimit = new DigitalInput(Constants.Arm.FRONT_LIMIT_PORT);
    backLimit = new DigitalInput(Constants.Arm.BACK_LIMIT_PORT);

    resetEncoders();

    rotaryEncoder.setDistancePerRotation(-360);

    goingForward = false;

    SmartDashboard.putData(this);
    SmartDashboard.putData(rotaryEncoder);
    SmartDashboard.putData(frontLimit);
    SmartDashboard.putData(backLimit);
  }

  public void rotate(double speed) {
    motor.set(MathUtil.clamp(speed, -0.7, 0.7));
  }

  public void goForward(double speed) {
    if (!isForward()) {
      goingForward = true;
      // Prevents arm from destroying platform
      double limitedSpeed = speed * limitorBasedOnRotation(this.getAngle());
      rotate(limitedSpeed);
    } else {
      stop();
    }
  }

  public void goBackwards(double speed) {
    // Speed is assumed to negative
    if (!isBack()) {
      // Prevents arm from destroying platform
      goingForward = false;
      double limitedSpeed = speed * limitorBasedOnRotation(this.getAngle());
      rotate(limitedSpeed);
    } else {
      stop();
    }
  }

  public void stop() {
    motor.set(0);
  }

  public double getDefaultMotorEncoderPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    //If the back limit is turned on, reset the encoders to 0
    if (!backLimit.get()) {
      resetEncoders();
    }
  }

  public boolean isForward() {
    return !frontLimit.get() || getAngle() >= Constants.Arm.FORWARD_LIMIT_ANGLE;
  }

  public boolean isBack() {
    return !backLimit.get() || getAngle() <= Constants.Arm.BACK_LIMIT_ANGLE;
  }

  public double getAngle() {
    return rotaryEncoder.getDistance();
  }

  public double limitorBasedOnRotation(double angle) {

    // Are we in the zone approaching the floor
    if (angle > Constants.Arm.BOUNDARY_FAST_MAXIMUM) {
      if (goingForward) {
        // Go slow as we are approaching the floor
        return Constants.Arm.ENTERING_ROTATION_SAFETY_ZONE_LIMIT;
      } else {
        return Constants.Arm.EXITING_ROTATION_SAFETY_ZONE_LIMIT;
      }
    }

    // Are we in the zone approaching the inside of the robot
    if (angle < Constants.Arm.BOUNDARY_FAST_MINIMUM) {
      if (goingForward) {
        return Constants.Arm.EXITING_ROTATION_SAFETY_ZONE_LIMIT;

      } else {
        // If we are moving towards the inside, then go slow
        return Constants.Arm.ENTERING_ROTATION_SAFETY_ZONE_LIMIT;
      }

    }

    // Returns a default multiplier of 1 if in middle zone
    return 1;
  }

  public void coast() {
    motor.setIdleMode(IdleMode.kCoast);
  }

  public void lock() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("GoingForward", () -> goingForward, null);
    builder.addBooleanProperty("IsForward", this::isForward, null);
    builder.addBooleanProperty("isBack", this::isBack, null);
    builder.addDoubleProperty("Arm default motor encoder: ", this::getDefaultMotorEncoderPosition, null);
    builder.addDoubleProperty("Rotary encoder angle: ", rotaryEncoder::getDistance, null);
    builder.addDoubleProperty("Arm angle the robot's using: ", this::getAngle, null);
    frontLimit.initSendable(builder);
    backLimit.initSendable(builder);

  }

  private void resetEncoders() {
    rotaryEncoder.reset();
    motor.getEncoder().setPosition(0);
  }
}
