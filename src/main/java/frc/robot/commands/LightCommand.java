// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class LightCommand extends CommandBase {
  private final DriveTrainSubsystem driveTrain;
  private double lightNumber;
  private double cyclesElapsed;
  /** Creates a new PurpleLightCommand. */
  public LightCommand(DriveTrainSubsystem driveTrain, double lightNumber) {
    this.driveTrain = driveTrain;
    this.lightNumber = lightNumber;
    cyclesElapsed = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setLights(lightNumber);
    cyclesElapsed++;
  }

  // Called once the command ends or is inte%rrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLights(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cyclesElapsed >= 50;
  }
}
