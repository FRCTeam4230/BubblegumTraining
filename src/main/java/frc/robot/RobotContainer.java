// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommand;
import frc.robot.Subsystem.DriveTrainSubsystem;


public class RobotContainer {

  private DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  private DriveCommand driveCommand = new DriveCommand(driveTrainSubsystem, xboxController::getLeftY, xboxController::getLeftX);
  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {

  }
  private void configureDefaultCommands() {
    driveTrainSubsystem.setDefaultCommand(driveCommand);
  }
  public Command getAutonomousCommand() {
    return null;
  }
}
