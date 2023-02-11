// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StaticFunctions;
import frc.robot.Constants.MotorID;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax motor;

  //color sensor in the future to know what we are carrying. (for auto and for dashbaord alerts)


  public IntakeSubsystem() {
    motor = StaticFunctions.initiateCANSparkMaxMotor.apply(MotorID.INTAKE_MOTOR_ID);
    
    SmartDashboard.putData(this);
  }

  public void setSpeed(double speed) {
    motor.set(MathUtil.clamp(speed, -.99, .99));
  }

  public void stop() {
    motor.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder){

  }
}
