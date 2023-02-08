// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmPIDConstants;
import frc.robot.Constants.MotorID;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.OutputCmd;
import frc.robot.commands.PIDCommandWithTolerance;
import frc.robot.commands.ArmPID;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Needed to pass in list because it uses a list in constructor
  private final DriveTrain driveTrain = new DriveTrain(
    Arrays.asList(MotorID.LEFT_1_MOTOR_ID, MotorID.LEFT_2_MOTOR_ID, MotorID.RIGHT_1_MOTOR_ID,
    MotorID.RIGHT_2_MOTOR_ID));

  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final XboxController driverController =
      new XboxController(Constants.OI.XBOX_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();

    //configure default commands
    configureDefaultCommands();
  }

  private void configureBindings() {
    //Buttons for intake
    new JoystickButton(driverController, 
    XboxController.Button.kLeftBumper.value).whileTrue(new IntakeCmd(intakeSubsystem));
    new JoystickButton(driverController, 
    XboxController.Button.kRightBumper.value).whileTrue(new OutputCmd(intakeSubsystem));

    //Buttons for arm
    new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new ArmPID(armSubsystem,
    Constants.ArmPositions.PICK_UP_FROM_GROUND));
    new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new ArmPID(armSubsystem,
    Constants.ArmPositions.PICK_UP_FROM_STATION));
    new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new ArmPID(armSubsystem,
    Constants.ArmPositions.BRING_IN));
    new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(new ArmPID(armSubsystem,
    Constants.ArmPositions.SCORE));



    //should cause robot to drive straight while the start button is held
    new JoystickButton(driverController, XboxController.Button.kStart.value).whileTrue(
        new PIDCommandWithTolerance(
          //Creates PID controller to pass into PID Command
            new PIDController(
                Constants.driveTrain.kStabilizationP,
                Constants.driveTrain.kStabilizationI,
                Constants.driveTrain.kStabilizationD),
            //Passes in measurement supplier
            // Close the loop on the turn rate
              driveTrain::getRobotPitch,
            // Passes in setpoint
            1,
            // Pipe the output to the turning controls
            output -> driveTrain.arcadeDrive(MathUtil.clamp(-output, -0.3, 0.3), 0),
            // Require the robot drive 
            driveTrain,
            //Position tolerance
            Constants.driveTrain.kPositionTolerance,
            //Velocity tolerance
            Constants.driveTrain.kVelocityTolerance));

    

  }

  private void configureDefaultCommands(){
    //Setting default commands
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, new Drive(driveTrain,driverController));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
