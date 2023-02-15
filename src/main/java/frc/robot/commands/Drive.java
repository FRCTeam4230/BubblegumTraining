// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private final DriveTrainSubsystem driveTrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;
  private final BooleanSupplier armOut;

  public Drive(DriveTrainSubsystem driveTrain, DoubleSupplier speed, DoubleSupplier rotation, BooleanSupplier armOut) {
    this.driveTrain = driveTrain;
    this.speed = speed;
    this.rotation = rotation;
    this.armOut = armOut;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //GET TEH SPEED /rotation here. 
    double wantedSpeed = speed.getAsDouble();
    double wantedRotation = rotation.getAsDouble();

    //Global motor limits
    if (armOut.getAsBoolean()){
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_ARM_OUT_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.SPEED_ARM_OUT_MULTIPLIER;
    }else{
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.SPEED_MULTIPLIER;
    }

    //now pass in the calcualted speed and rotation
    driveTrain.arcadeDrive(-1 * wantedSpeed, wantedRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
