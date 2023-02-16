// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorID;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveToChargeStation;
import frc.robot.commands.HoldArmCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SetDriveTrainMotorIdleMode;
import frc.robot.commands.ArmBackwardCmd;
import frc.robot.commands.ArmForwardCmd;
import frc.robot.commands.ArmPID;
import frc.robot.commands.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
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
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(
      Arrays.asList(MotorID.LEFT_1_MOTOR_ID, MotorID.LEFT_2_MOTOR_ID, MotorID.RIGHT_1_MOTOR_ID,
          MotorID.RIGHT_2_MOTOR_ID));

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Commands
  private DoubleSupplier intakeSupplier = () -> driverController.getLeftTriggerAxis()
      - driverController.getRightTriggerAxis();
  private final IntakeCmd intakeCommand = new IntakeCmd(intakeSubsystem, intakeSupplier);


  private Command basicAutoCommand = 
    (new DriveToChargeStation(driveTrain, () -> driveTrain.getPitch()))
    .andThen(new Balance(driveTrain))
    .andThen(() -> driveTrain.lock());


   private Command bringInArm = 
    new ArmPID(armSubsystem, () -> Constants.ArmPositions.BRING_IN);

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
     * Constants.ArmPositions.BRING_IN)); */

      new JoystickButton(driverController, XboxController.Button.kX.value)
      .onTrue(
              new ArmPID(armSubsystem, () -> Constants.ArmPositions.SCORE)
              .andThen(
                new HoldArmCommand(armSubsystem, Constants.ArmPositions.SCORE)
                .alongWith( new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED).withTimeout(2)))      
      );


      new JoystickButton(driverController, XboxController.Button.kY.value)
        .onTrue(bringInArm);


     

    // Buttons for moving arm
    new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).whileTrue(new ArmBackwardCmd(armSubsystem));
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value).whileTrue(new ArmForwardCmd(armSubsystem));

    // test auto.. use back button
    new JoystickButton(driverController, XboxController.Button.kBack.value)
        .onTrue(basicAutoCommand);

    // new JoystickButton(driverController, XboxController.Button.kStart.value)
    //     .whileTrue(new Balance(driveTrain));

        new JoystickButton(driverController, XboxController.Button.kA.value)
        .whileTrue(DriveDistance.create(driveTrain, Constants.AutoConstants.DISTANCE_TO_CHARGE_STATION));
  }



  private void configureDefaultCommands() {
    // Setting default commands
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, getTeleopCommand());
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
    //need to tell the drive command about the arm position
    return new Drive(driveTrain, () -> driverController.getLeftY(), () ->  driverController.getRightX(), () -> !armSubsystem.isBack(), 
    () -> armSubsystem.getAngle());
  }

}
