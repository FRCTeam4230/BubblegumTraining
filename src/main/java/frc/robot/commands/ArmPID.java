// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPID extends CommandBase {
  private final ArmSubsystem armSubsystem;
  
  private final PIDController pidController;
  private final double target;

  public ArmPID(ArmSubsystem armSubsystem, double target) {
    this.armSubsystem = armSubsystem;

    pidController = new PIDController(Constants.ArmPIDConstants.kP, Constants.ArmPIDConstants.kI, 
    Constants.ArmPIDConstants.kD);
    pidController.setTolerance(Constants.ArmPIDConstants.POSITION_TOLERANCE);

    this.target = target;

    addRequirements(armSubsystem);

    SmartDashboard.putData(this);

  }


  @Override
  public void initialize() {
    pidController.setSetpoint(target);
  }


  @Override
  public void execute() {
    double output = pidController.calculate(armSubsystem.getPosition());

    //Puts output into a range
    output = MathUtil.clamp(output, -Constants.ArmPIDConstants.RANGE, Constants.ArmPIDConstants.RANGE);
    armSubsystem.setAngle(output, true);
  }


  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }


  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP: ", pidController::getP, pidController::setP);
    builder.addDoubleProperty("kI: ", pidController::getI, pidController::setI);
    builder.addDoubleProperty("kD: ", pidController::getD, pidController::setD);
    builder.addDoubleProperty("Encoder Degrees: ", armSubsystem::getPosition, null);
  
  }
}
