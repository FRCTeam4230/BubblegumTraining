// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPID extends PIDCommand {
  protected final ArmSubsystem armSubsystem;
  

  //Takes in arm subsystem and the target angle
  public ArmPID(ArmSubsystem armSubsystem, DoubleSupplier targetAngleSupplier) {
    super(
      new PIDController(Constants.ArmPIDConstants.kP, Constants.ArmPIDConstants.kI, 
    Constants.ArmPIDConstants.kD),
    armSubsystem::getAngle,
    targetAngleSupplier::getAsDouble,
    output -> {
      if(output > 0) {
        armSubsystem.goForward(output);
      } else if(output < 0) {
        armSubsystem.goBackwards(output);
      }
    },
    armSubsystem);

    getController().setTolerance(Constants.ArmPIDConstants.POSITION_TOLERANCE);
    



    this.armSubsystem = armSubsystem;

    SmartDashboard.putData(this);


  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  public boolean isFinished() {
    return getController().atSetpoint();
  }




  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // builder.addDoubleProperty("kP: ", pidController::getP, pidController::setP);
    // builder.addDoubleProperty("kI: ", pidController::getI, pidController::setI);
    // builder.addDoubleProperty("kD: ", pidController::getD, pidController::setD);
    // builder.addDoubleProperty("Encoder Degrees: ", armSubsystem::getPosition, null);
  
  }
}
