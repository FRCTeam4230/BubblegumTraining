// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorID;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.arm.ArmBackwardCmd;
import frc.robot.commands.arm.ArmForwardCmd;
import frc.robot.commands.arm.ArmPIDAgainstGravity;
import frc.robot.commands.arm.ArmPIDWithGravity;
import frc.robot.commands.arm.PIDHoldArmCommand;
import frc.robot.commands.auto.LongAutoCommand;
import frc.robot.commands.auto.MiddleAutoCommand;
import frc.robot.commands.auto.ScoreTopAutoCommand;
import frc.robot.commands.auto.ShortAutoCommand;
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

public class RobotContainer {
  private final XboxController driverController = new XboxController(Constants.OI.DRIVER_XBOX_PORT);
  private final XboxController intakeController = new XboxController(Constants.OI.INTAKE_XBOX_PORT);

  //Allows us to pick our auto routine through shuffleboard, so we don't have to re-depoly code
  //each time we want to use a different auto routine
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Subsystems
  // Needed to pass in list because it uses a list in constructor
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(
          Arrays.asList(MotorID.LEFT_1_MOTOR_ID, MotorID.LEFT_2_MOTOR_ID, MotorID.RIGHT_1_MOTOR_ID,
                  MotorID.RIGHT_2_MOTOR_ID));

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Commands
  private final DoubleSupplier intakeSupplier = () -> driverController.getLeftTriggerAxis()
          - driverController.getRightTriggerAxis();
  private final IntakeCmd intakeCommand = new IntakeCmd(intakeSubsystem, intakeSupplier);

  private final IntakeCmd pickUpCone = new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd outputCone = new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd pickUpCube = new IntakeCmd(intakeSubsystem, () -> Constants.Intake.INTAKE_SPEED);
  private final IntakeCmd outputCube = new IntakeCmd(intakeSubsystem, () -> -Constants.Intake.INTAKE_SPEED);

  private final ArmPIDWithGravity bringInArm = new ArmPIDWithGravity(armSubsystem,
          Constants.ArmPositions.BRING_IN);
  private final ArmPIDWithGravity pickUpFromGround = new ArmPIDWithGravity(armSubsystem,
          Constants.ArmPositions.PICK_UP_FROM_GROUND);
  private final ArmPIDAgainstGravity pickUpFromStation = new ArmPIDAgainstGravity(armSubsystem,
          Constants.ArmPositions.PICK_UP_FROM_STATION);
  private final ArmPIDAgainstGravity scoreTop = new ArmPIDAgainstGravity(armSubsystem,
          Constants.ArmPositions.SCORE_TOP);
  private final ArmPIDAgainstGravity scoreMiddle = new ArmPIDAgainstGravity(armSubsystem,
          Constants.ArmPositions.SCORE_MIDDLE);

  private final MiddleAutoCommand middleAutoCommand = new MiddleAutoCommand(armSubsystem, intakeSubsystem, driveTrain);
  private final LongAutoCommand longAutoCommand = new LongAutoCommand(driveTrain, armSubsystem, intakeSubsystem);
  private final ScoreTopAutoCommand scoreTopAutoCommand = new ScoreTopAutoCommand(armSubsystem, intakeSubsystem);
  private final ShortAutoCommand shortAutoCommand = new ShortAutoCommand(driveTrain, armSubsystem, intakeSubsystem);

  private final ArmForwardCmd manualArmForward = new ArmForwardCmd(armSubsystem);
  private final ArmBackwardCmd manualArmBackward = new ArmBackwardCmd(armSubsystem);

  private final Balance balance = new Balance(driveTrain);

  public RobotContainer() {
    //Auto chooser is the thing that lets us change auto routines through shuffleboard
    configureAutoChooser();

    configureCamera();

    configureDefaultCommands();

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureCamera() {
    CameraServer.startAutomaticCapture();
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Score Top & Drive Long", longAutoCommand);
    autoChooser.addOption("Score Top & Drive short", shortAutoCommand);
    autoChooser.addOption("Score top and middle balance", middleAutoCommand);
    autoChooser.addOption("Score Top", scoreTopAutoCommand);
    SmartDashboard.putData("Autonomous routine", autoChooser);
  }


  private void configureBindings() {
    // Buttons for automated arm movement

    //Start button brings in the arm
    new JoystickButton(driverController,
            XboxController.Button.kStart.value).onTrue(
            bringInArm);
    // Button A picks up from ground
    new JoystickButton(driverController,
            XboxController.Button.kA.value).onTrue(
            pickUpFromGround);

    //Button X picks up from station
    new JoystickButton(driverController,
            XboxController.Button.kX.value).onTrue(
            pickUpFromStation);

    //Button Y scores top row
    new JoystickButton(driverController,
            XboxController.Button.kY.value).onTrue(
            scoreTop);

    //B button for middle row
    new JoystickButton(driverController,
            XboxController.Button.kB.value).onTrue(
            scoreMiddle);

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
    armSubsystem.setDefaultCommand(new PIDHoldArmCommand(armSubsystem));
  }

  public Command getAutonomousCommand() {
    driveTrain.zeroHeading();
    return autoChooser.getSelected();
  }

  /*
   * teleop command. disable the brakes after auto
   */
  public Command getTeleopCommand() {
    driveTrain.coast();
    // need to tell the drive command about the arm position
    return new Drive(driveTrain, driverController::getLeftY, driverController::getRightX, armSubsystem::getAngle);
  }
}
