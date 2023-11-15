// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDAgainstGravity extends PIDCommand {
  protected final ArmSubsystem armSubsystem;


  //Takes in arm subsystem and the target angle
  public ArmPIDAgainstGravity(ArmSubsystem armSubsystem, double targetAngle) {
    super(
            new PIDController(Constants.ArmPIDConstants.P_AGAINST_GRAVITY,
                    Constants.ArmPIDConstants.I_AGAINST_GRAVITY,
                    Constants.ArmPIDConstants.D_AGAINST_GRAVITY),
            armSubsystem::getAngle,
            targetAngle,
            output -> {
              if (output > 0) {
                armSubsystem.goForward(output);
              } else if (output < 0) {
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

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
