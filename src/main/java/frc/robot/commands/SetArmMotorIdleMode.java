// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmMotorIdleMode extends CommandBase {
  private ArmSubsystem armSubsystem;
  private boolean lock;
  /** Creates a new SetArmMotorIdleMode. */
  public SetArmMotorIdleMode(ArmSubsystem armSubsystem, boolean lock) {
    this.armSubsystem = armSubsystem;
    this.lock = lock;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lock) {
      armSubsystem.lock();
    } else {
      armSubsystem.coast();
    }
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
