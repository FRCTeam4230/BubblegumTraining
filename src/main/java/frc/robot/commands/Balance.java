// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Balance extends PIDCommand {
  private DriveTrainSubsystem driveTrain;
  public Balance(DriveTrainSubsystem driveTrain) {
    super(
      new PIDController(
        Constants.DriveTrain.CHARGE_STATION_P,
        Constants.DriveTrain.CHARGE_STATION_I,
        Constants.DriveTrain.CHARGE_STATION_D),
  //Passes in measurement supplier
    driveTrain::getPitch,
  // Passes in setpoint
  driveTrain::getSetPoint,
  // Pipe the output to the turning controls
  output -> driveTrain.arcadeDrive(MathUtil.clamp(-output , -Constants.DriveTrain.PID_CLAMP_RANGE, Constants.DriveTrain.PID_CLAMP_RANGE), 0),
  // Require the robot driveTrain
  driveTrain);

  getController().setTolerance(Constants.DriveTrain.kPositionTolerance);

  this.driveTrain = driveTrain;
  SmartDashboard.putData(this);
  }

  @Override
  public void initialize() {
    driveTrain.lock();
  }

  @Override
  public void execute() {
      super.execute();
  }

  // Returns false because we don't want the robot to stop balancing
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
      driveTrain.coast();
  }
}
