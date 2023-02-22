// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//Auto command for when we are in the middle
//Deposit cone, drive over charge station to get robot mobility points
//drive back to charge station, balance
public class MiddleAutoCommand extends SequentialCommandGroup {
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  DriveTrainSubsystem driveTrain;

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
            .withTimeout(0.5),
        new ArmPIDWithGravity(armSubsystem, () -> Constants.ArmPositions.BRING_IN)
            .withTimeout(3.5),
        new DrivePastChargeStation(driveTrain, () -> driveTrain.getPitch()),
        new DriveBackToChargeStation(driveTrain), 
        new Balance(driveTrain),
        new SetDriveTrainMotorIdleMode(driveTrain, true)
        );
  }
}
