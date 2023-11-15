// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MiddleAutoCommand extends SequentialCommandGroup {
  public MiddleAutoCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem
          , DriveTrainSubsystem driveTrainSubsystem) {
    addCommands(
            new ScoreTopAutoCommand(armSubsystem, intakeSubsystem),
            new DriveToChargeStation(driveTrainSubsystem)
                    .withTimeout(5),
            new Balance(driveTrainSubsystem)
                    .withTimeout(10)
    );
  }
}
