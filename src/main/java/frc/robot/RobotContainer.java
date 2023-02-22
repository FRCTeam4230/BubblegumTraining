// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MotorID;
import frc.robot.commands.Drive;
import frc.robot.commands.HoldArmCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.LightCommand;
import frc.robot.commands.MiddleAutoCommand;
import frc.robot.commands.RightAutoCommand;
import frc.robot.commands.ArmBackwardCmd;
import frc.robot.commands.ArmForwardCmd;
import frc.robot.commands.ArmPIDAgainstGravity;
import frc.robot.commands.ArmPIDWithGravity;
import frc.robot.commands.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cameraserver.CameraServer;
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
  private final XboxController driverController = new XboxController(Constants.OI.DRIVER_XBOX_PORT);
  private final XboxController intakeController = new XboxController(Constants.OI.INTAKE_XBOX_PORT);


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

  private final IntakeCmd pickUpCone = new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd outputCone = new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd pickUpCube = new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd outputCube = new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED);

  //Testing stuff with lights
  private final LightCommand purpleLight = new LightCommand(driveTrain, Constants.LightNumbers.PURPLE);
  private final LightCommand yellowLight = new LightCommand(driveTrain, Constants.LightNumbers.YELLOW);

  private final ArmPIDWithGravity bringInArm = new ArmPIDWithGravity(armSubsystem,
      () -> Constants.ArmPositions.BRING_IN);
  private final ArmPIDWithGravity pickUpFromGround = new ArmPIDWithGravity(armSubsystem,
      () -> Constants.ArmPositions.PICK_UP_FROM_GROUND);
  private final ArmPIDAgainstGravity pickUpFromStation = new ArmPIDAgainstGravity(armSubsystem,
      () -> Constants.ArmPositions.PICK_UP_FROM_STATION);
  private final ArmPIDAgainstGravity scoreTop = new ArmPIDAgainstGravity(armSubsystem,
      () -> Constants.ArmPositions.SCORE_TOP);
  private final ArmPIDAgainstGravity scoreMiddle = new ArmPIDAgainstGravity(armSubsystem,
      () -> Constants.ArmPositions.SOCRE_MIDDLE);

  private final HoldArmCommand holdBringInArm = new HoldArmCommand(armSubsystem, Constants.ArmPositions.BRING_IN);
  private final HoldArmCommand holdPickUpFromGround = new HoldArmCommand(armSubsystem,
      Constants.ArmPositions.PICK_UP_FROM_GROUND);
  private final HoldArmCommand holdPickUpFromStation = new HoldArmCommand(armSubsystem,
      Constants.ArmPositions.PICK_UP_FROM_STATION);
  private final HoldArmCommand holdScoreTop = new HoldArmCommand(armSubsystem, Constants.ArmPositions.SCORE_TOP);
  private final HoldArmCommand holdScoreMiddle = new HoldArmCommand(armSubsystem, Constants.ArmPositions.SOCRE_MIDDLE);

//   private final MiddleAutoCommand autoCommand = new MiddleAutoCommand(armSubsystem, intakeSubsystem, driveTrain);
  private final RightAutoCommand autoCommand = new RightAutoCommand(driveTrain, armSubsystem, intakeSubsystem);
  private final ArmForwardCmd manualArmForward = new ArmForwardCmd(armSubsystem);
  private final ArmBackwardCmd manualArmBackward = new ArmBackwardCmd(armSubsystem);

  private final Balance balance = new Balance(driveTrain);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();

    CameraServer.startAutomaticCapture();


    // configure default commands
    configureDefaultCommands();

  }

  private void configureBindings() {

    // Buttons for automated arm movement

    //Start button brings in the arm
    new JoystickButton(driverController,
        XboxController.Button.kStart.value).onTrue(
            bringInArm
                .andThen(holdBringInArm));

    //Button A picks up from ground
    new JoystickButton(driverController,
        XboxController.Button.kA.value).onTrue(
            pickUpFromGround
                .andThen(holdPickUpFromGround));

    //Button X picks up from station
    new JoystickButton(driverController,
        XboxController.Button.kX.value).onTrue(
            pickUpFromStation
                .andThen(holdPickUpFromStation));

    //Button Y scores top row
    new JoystickButton(driverController,
        XboxController.Button.kY.value).onTrue(
            scoreTop
                .andThen(holdScoreTop));

    //B button for middle row
    new JoystickButton(driverController,
        XboxController.Button.kB.value).onTrue(
            scoreMiddle
                .andThen(holdScoreMiddle));

    //Buttons for moving arm manually
    new JoystickButton(driverController,
    XboxController.Button.kLeftBumper.value).whileTrue(manualArmBackward);
    new JoystickButton(driverController,
    XboxController.Button.kRightBumper.value).whileTrue(manualArmForward);

    //Back button to balance on the charge station
    new JoystickButton(driverController,
    XboxController.Button.kBack.value).whileTrue(balance);

    // Buttons for intake controller

    //Y button for pickup cone
    new JoystickButton(intakeController, XboxController.Button.kY.value)
        .whileTrue(pickUpCone);

    // A button for outputing cone
    new JoystickButton(intakeController, XboxController.Button.kA.value)
        .whileTrue(outputCone);

    // X button for picking up cube
    new JoystickButton(intakeController, XboxController.Button.kX.value)
        .whileTrue(pickUpCube);

    // B button for outputing cube
    new JoystickButton(intakeController, XboxController.Button.kB.value)
        .whileTrue(outputCube);

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
    return autoCommand;
  }

  /*
   * teleop command. disable the brakes after auto
   */
  public Command getTeleopCommand() {
    driveTrain.coast();
    // need to tell the drive command about the arm position
    return new Drive(driveTrain, () -> driverController.getLeftY(), () -> driverController.getRightX(),
        () -> armSubsystem.getAngle());
  }

}
