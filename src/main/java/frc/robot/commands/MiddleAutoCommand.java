// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.Balance;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleAutoCommand extends SequentialCommandGroup {
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  DriveTrainSubsystem driveTrain;

  /** Creates a new MiddleAutoCommand. */
  public MiddleAutoCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
      DriveTrainSubsystem driveTrain) {
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmPIDAgainstGravity(
            armSubsystem, () -> Constants.ArmPositions.SCORE_TOP)
            .withTimeout(3),
        new HoldArmCommand(armSubsystem, Constants.ArmPositions.SCORE_TOP)
            .withTimeout(2)
            .alongWith(new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED))
            .withTimeout(1),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.BRING_IN)
            .withTimeout(3),
        new DriveToChargeStation(driveTrain, () -> driveTrain.getPitch(), () -> 32.0), //INCHES TO CHARGE STATION --> make constant
        new Balance(driveTrain),
        new SetDriveTrainMotorIdleMode(driveTrain, true)
        );

        // (() -> driveTrain.lock()).withTimeout(2));
        // new ArmPIDAgainstGravity(armSubsystem, () -> 35));
  }
}
