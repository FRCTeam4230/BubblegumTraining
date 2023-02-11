// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorID;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SetDriveTrainMotorIdleMode;
import frc.robot.commands.ArmBackwardCmd;
import frc.robot.commands.ArmForwardCmd;
import frc.robot.commands.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driverController = new XboxController(Constants.OI.XBOX_PORT);
  // Subsystems
  // Needed to pass in list because it uses a list in constructor
  private final DriveTrain driveTrain = new DriveTrain(
      Arrays.asList(MotorID.LEFT_1_MOTOR_ID, MotorID.LEFT_2_MOTOR_ID, MotorID.RIGHT_1_MOTOR_ID,
          MotorID.RIGHT_2_MOTOR_ID));

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Commands
  private DoubleSupplier intakeSupplier = () -> driverController.getLeftTriggerAxis()
      - driverController.getRightTriggerAxis();
  private final IntakeCmd intakeCommand = new IntakeCmd(intakeSubsystem, intakeSupplier);


  private Command basicAutoCommand =       
    new SetDriveTrainMotorIdleMode(driveTrain, true)
    .andThen(DriveDistance.create(driveTrain, Constants.AutoConstants.DISTANCE_TO_CHARGE_STATION))
    .andThen(new Balance(driveTrain));



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    // configure default commands
    configureDefaultCommands();
  }

  private void configureBindings() {

    /*
     * new JoystickButton(driverController,
     * XboxController.Button.kA.value).onTrue(new ArmPID(armSubsystem,
     * Constants.ArmPositions.PICK_UP_FROM_GROUND));
     * new JoystickButton(driverController,
     * XboxController.Button.kB.value).onTrue(new ArmPID(armSubsystem,
     * Constants.ArmPositions.PICK_UP_FROM_STATION));
     * new JoystickButton(driverController,
     * XboxController.Button.kY.value).onTrue(new ArmPID(armSubsystem,
     * Constants.ArmPositions.BRING_IN));
     * new JoystickButton(driverController,
     * XboxController.Button.kX.value).onTrue(new ArmPID(armSubsystem,
     * Constants.ArmPositions.SCORE));
     */

    // Buttons for moving arm
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).whileTrue(new ArmForwardCmd(armSubsystem));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileTrue(new ArmBackwardCmd(armSubsystem));

    // test auto.. use back button
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(basicAutoCommand);

    new JoystickButton(driverController, XboxController.Button.kStart.value)
        .whileTrue(new Balance(driveTrain));

  }

  private void configureDefaultCommands() {
    // Setting default commands
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new Drive(driveTrain, driverController::getLeftY,
        driverController::getRightX));
    CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, intakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return basicAutoCommand;
  }

  /*
   * teleop command. disable the brakes after auto
   */
  public Command getTeleopCommand() {
    driveTrain.coast();
    return new Drive(driveTrain, driverController::getLeftY, driverController::getRightY);
  }

}
