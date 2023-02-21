// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAutoCommand extends SequentialCommandGroup {
  /** Creates a new RightAutoCommand. */
  private DriveTrainSubsystem driveTrain;
  private ArmSubsystem armSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public RightAutoCommand(DriveTrainSubsystem driveTrain, ArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmPIDAgainstGravity(
            armSubsystem, () -> Constants.ArmPositions.SCORE_TOP)
            .withTimeout(3),
        new HoldArmCommand(armSubsystem, Constants.ArmPositions.SCORE_TOP)
            .withTimeout(2)
            .alongWith(new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED))
            .withTimeout(0.5),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.BRING_IN)
            .withTimeout(3),
        new DriveDistance(driveTrain, -0.5)
            .withTimeout(0.5),
        //EVerything below here needs refining
        new PIDTurn(driveTrain, 180)
            .withTimeout(2),
        new DriveDistance(driveTrain, 1)
        .withTimeout(3.5),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.PICK_UP_FROM_GROUND)
        .withTimeout(2)
        .alongWith(new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED)
        .withTimeout(0.5)),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.BRING_IN)
        .withTimeout(2)
        );
  }
}
