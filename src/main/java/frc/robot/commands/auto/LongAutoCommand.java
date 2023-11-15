// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//Auto command for driving a longer distance to leave community
//Deposit cone, drive out of community up to another element, put arm down so during teleop it's easy to pick up element
public class LongAutoCommand extends SequentialCommandGroup {
  public LongAutoCommand(DriveTrainSubsystem driveTrain, ArmSubsystem armSubsystem,
                         IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            new ScoreTopAutoCommand(armSubsystem, intakeSubsystem),
            new DriveDistance(driveTrain, -(Constants.AutoConstants.LEFT_START_TO_COMMUNITY_LINE + Constants.CompetitionRobot.length))
                    .withTimeout(3)
    );
  }
}
