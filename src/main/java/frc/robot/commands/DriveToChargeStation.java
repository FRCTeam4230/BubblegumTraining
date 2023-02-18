// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveToChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation. */
  private final DriveTrainSubsystem driveTrain;
  private final DoubleSupplier pitchSupplier;
  private boolean AtChargeStation;
  private double cyclesElapsed;
  private final PIDController pidController;

  public DriveToChargeStation(DriveTrainSubsystem driveTrain, DoubleSupplier pitchSupplier) {
    this.driveTrain = driveTrain;
    this.pitchSupplier = pitchSupplier;
    AtChargeStation = false;
    pidController = new PIDController(
      Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI, Constants.DriveTrain.TURN_KD);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    pidController.setSetpoint(0);
    pidController.setTolerance(1.5);
    cyclesElapsed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(driveTrain.getHeading());

    //Drive at half speed
    //Uses PID controller with robot heading to make sure the robot is going straight
    driveTrain.arcadeDrive(-0.5, MathUtil.clamp(output, -0.2, 0.2));

    if(AtChargeStation) {
      //If the robot is at the charge station, add 1 to cyclesElapsed
      //cyclesElasped is used in the isFinished method to determine when the command should end
      cyclesElapsed += 1;
    }

    if(pitchSupplier.getAsDouble() > Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH) {
      //If the pitch goes past 15 degrees, we know that the robot is at least partially on the charge station
      AtChargeStation = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cyclesElapsed >= 10) {
    //Each cycle is 20 ms long. 20 cyclesElapsed means 0.4 seconds. Once the robot
    //reaches the charge station, keep going for 0.4 seconds, then end the command
      return true;
    }
    return false;
  }
}
