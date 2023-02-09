// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private final DriveTrain driveTrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;

  public Drive(DriveTrain driveTrain, DoubleSupplier speed, DoubleSupplier rotation) {
    this.driveTrain = driveTrain;
    this.speed = speed;
    this.rotation = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(speed.getAsDouble() * -1 * Constants.driveTrain.SPEED_MULTIPLIER, 
    rotation.getAsDouble() * Constants.driveTrain.ROTATION_MULTIPLIER);
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
