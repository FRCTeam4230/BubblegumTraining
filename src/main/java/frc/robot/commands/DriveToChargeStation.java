// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveToChargeStation extends CommandBase {
  private DriveTrainSubsystem driveTrain;
  private PIDController rotationPIDController;
  private boolean onChargeStation;
  /** Creates a new DriveToChargeStation. */
  public DriveToChargeStation(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    rotationPIDController = new PIDController(
      Constants.DriveTrain.TURN_KP,
      Constants.DriveTrain.TURN_KI,
      Constants.DriveTrain.TURN_KD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    onChargeStation = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    rotationPIDController.setSetpoint(0);
    rotationPIDController.setTolerance(1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = rotationPIDController.calculate(driveTrain.getRawHeading());
    if(driveTrain.getPitch() <= -Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH){
      onChargeStation = true;
    }
  if(onChargeStation){
    //Go slow when on the charge station
    driveTrain.arcadeDrive(-0.4, MathUtil.clamp(output, -0.2, 0.2));
  } else {
    //Go fast when going on the charge station
    driveTrain.arcadeDrive(-0.6, MathUtil.clamp(output, -0.2, 0.2));
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onChargeStation && driveTrain.getPitch() >= -6;
  }
}
