// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//Auto command for when we are in the middle
//Deposit cone, drive over charge station to get robot mobility points
//drive back to charge station, balance
public class MiddleAutoCommandAdvanced extends SequentialCommandGroup {
  ArmSubsystem armSubsystem;
  IntakeSubsystem intakeSubsystem;
  DriveTrainSubsystem driveTrain;

  public MiddleAutoCommandAdvanced(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem,
      DriveTrainSubsystem driveTrain) {
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.driveTrain = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ScoreTopAutoCommand(armSubsystem, intakeSubsystem),
        new DrivePastChargeStation(driveTrain)
        .withTimeout(3),
        new WaitCommand(0.5),
        new DriveBackToChargeStation(driveTrain)
        .withTimeout(3), 
        new Balance(driveTrain)
        );
  }
}
