// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PIDTurn extends CommandBase {
  DriveTrainSubsystem driveTrain;
  PIDController pidController;

  public PIDTurn(DriveTrainSubsystem driveTrain, double targetAngle) {
    this.driveTrain = driveTrain;
    //Constants need tuning
    pidController = new PIDController(
      Constants.AutoConstants.PID_TURN_P,
      Constants.AutoConstants.PID_TURN_I,
      Constants.AutoConstants.PID_TURN_D);

    pidController.setSetpoint(targetAngle);
    pidController.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(driveTrain.getRawHeading());
    driveTrain.arcadeDrive(0, MathUtil.clamp(output, -0.5, 0.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
