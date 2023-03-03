// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveBackToChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation. */
  private final DriveTrainSubsystem driveTrain;
  private double cyclesElapsed;
  private final PIDController rotationPidController;

  public DriveBackToChargeStation(DriveTrainSubsystem driveTrain) {
    super();

    this.driveTrain = driveTrain;
    
    rotationPidController = new PIDController(
      Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI, Constants.DriveTrain.TURN_KD);
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    rotationPidController.setSetpoint(0);
    rotationPidController.setTolerance(1.5);
    cyclesElapsed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Uses PID controller with robot heading to make sure the robot is going straight
    double rotation = rotationPidController.calculate(driveTrain.getHeading());

    driveTrain.arcadeDrive(0.5, MathUtil.clamp(rotation, -0.2, 0.2));
    cyclesElapsed += 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //After the robot has driven backwards for 20 cycles, end command
    return cyclesElapsed >= 999;
  }
}