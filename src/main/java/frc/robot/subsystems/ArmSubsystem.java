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
  private final DutyCycleEncoder encoder;
  private final CANSparkMax motor;
  private final DigitalInput frontLimit;
  private final DigitalInput backLimit;
  private boolean goingForward;
  

  public ArmSubsystem() {
    motor = StaticFunctions.initiateCANSparkMaxMotor.apply(MotorID.ARM_MOTOR_ID);

    encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
    frontLimit = new DigitalInput(Constants.Arm.FRONT_LIMIT_PORT);
    backLimit = new DigitalInput(Constants.Arm.BACK_LIMIT_PORT);

    resetEncoders();

    encoder.setDistancePerRotation(-360);
    
    goingForward = false;

    SmartDashboard.putData(this);
    SmartDashboard.putData(encoder);
    SmartDashboard.putData(frontLimit);
    SmartDashboard.putData(backLimit);
  }

  private void rotate(double speed){
    motor.set(MathUtil.clamp(speed,-0.55,0.55));
  }

  public void goForward(double speed){
    if (!isForward()){
      goingForward = true;
      //Prevents arm from destroying platform
      double limitedSpeed = speed * limitorBasedOnRotation(this.getAngle());
      rotate(limitedSpeed);
    }else{
      stop();
    }
  }

  public void goBackwards(double speed){
    //Speed is assumed to negative
    if (!isBack()){
      //Prevents arm from destroying platform
      goingForward = false;
      double limitedSpeed = speed * limitorBasedOnRotation(this.getAngle());
      rotate(limitedSpeed);
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
    if((isBack() && !goingForward) || (isForward() && goingForward)) {
      // stop();
    }
  }

  public boolean isForward() {
    return !frontLimit.get() || getAngle() >= Constants.Arm.FORWARD_LIMIT_ANGLE;
  }

  public boolean isBack() {
    return !backLimit.get()|| getAngle() <= Constants.Arm.BACK_LIMIT_ANGLE;
  }

  public double getAngle() {
    //Check which value to get from encoder
    return encoder.getDistance();
  }

  public boolean getDirection() {
    return goingForward;
  }

  public double limitorBasedOnRotation(double angle) {

    //Are we in the zone approaching the floor
    if(angle > Constants.Arm.BOUNDARY_FAST_MAXIMUM) {
      if(goingForward) {
        //Go slow as we are approaching the floor
        return Constants.Arm.ENTERING_ROTATION_SAFETY_ZONE_LIMIT;
      } else {
      return Constants.Arm.EXITING_ROTATION_SAFETY_ZONE_LIMIT;
      }
    }

    //Are we in the zone approaching the inside of the robot
    if(angle < Constants.Arm.BOUNDARY_FAST_MINIMUM) {
      if(goingForward) {
      return Constants.Arm.EXITING_ROTATION_SAFETY_ZONE_LIMIT;

      } else {
      //If we are moving towards the inside, then go slow
      return Constants.Arm.ENTERING_ROTATION_SAFETY_ZONE_LIMIT * 0.7;

      }

    }

    //Returns a default multiplier of 1, hopefully this value is never returned, but we needed to add a default value
    //to avoid error
    return 1;
  }


  public int getZone() {
    //getZone returns the zone the arm is currently in
    //1 means we are in arm out zone
    //2 means we are in arm inside zone
    //3 means we are in arm up zone

    //Are we in the zone approaching the floor
    if(getAngle() > Constants.DriveTrain.ARM_OUT_BOUNDARY) {
      return 1;
    } else if(getAngle() < Constants.DriveTrain.ARM_IN_BOUNDARY) {
      return 2;
    } else {
      return 3;
    }
  }


  //Right now arm is only holding for preset locations
  public void holdAgainstGravity(){
    double speed = 0.02 * (isPastMiddle() ? -1 : 1);
    motor.set(speed);
  }

  public boolean isPastMiddle(){
    return getAngle() > Constants.Arm.FORWARD_LIMIT_ANGLE / 2; 
  }

  public void coast() {
    motor.setIdleMode(IdleMode.kCoast);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);


    builder.addBooleanProperty("GoingForward", () -> goingForward, null);
    builder.addBooleanProperty("IsForward", this::isForward, null);
    builder.addBooleanProperty("isBack", this::isBack, null);
    builder.addDoubleProperty("Arm motor encoder: ", this::getMotorEncoderPosition, null);
    builder.addDoubleProperty("Arm angle: ", this::getAngle, null);
    encoder.initSendable(builder);
    frontLimit.initSendable(builder);
    backLimit.initSendable(builder);

  }

  private void resetEncoders() {
    encoder.reset();
    motor.getEncoder().setPosition(0);
  }
}
