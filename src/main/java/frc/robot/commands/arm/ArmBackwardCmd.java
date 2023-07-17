// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;


import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;


public class ArmBackwardCmd extends ArmForwardCmd {

  public ArmBackwardCmd(ArmSubsystem armSubsystem) {
    super(armSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.goBackwards(Constants.Arm.ARM_SPEED * -1);
  }
}
