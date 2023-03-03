// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//Auto command for when we are on the left side
//Deposit cone, drive out of community up to another element, put arm down so during teleop it's easy to pick up element
public class LeftAutoCommand extends SequentialCommandGroup {
  public LeftAutoCommand(DriveTrainSubsystem driveTrain, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ScoreTopAutoCommand(armSubsystem, intakeSubsystem),
        new DriveDistance(driveTrain, -165)
        .withTimeout(4),
        new PIDTurn(driveTrain, 180)
            .withTimeout(1),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.PICK_UP_FROM_GROUND + 3)
        .withTimeout(5)
        .alongWith(new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED)
        .withTimeout(0.5))
        );
  }
}
