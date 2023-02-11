// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {
  private final DriveTrain driveTrain;
  private final ArmSubsystem armSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final Double distanceFromStartToScale = 62.0; //inches

  /*
   * Things to do for auto:
   * 1. completely leave community at any point = 3 points
   * 2. Score --> bottom = 3, middle = 4, top = 6
   * 3. Dock and engage --> dock = 8, dock and engage = 12
   * Docked means touching only the charging station
   * Engaged means the station is level and all other robots contacting the charge station are docked
   */
  public AutoCommand(DriveTrain driveTrain, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    this.driveTrain = driveTrain;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      DriveDistance.create(driveTrain, Constants.AutoConstants.DISTANCE_TO_CHARGE_STATION).andThen(
      new Balance(driveTrain)));
  }


}
