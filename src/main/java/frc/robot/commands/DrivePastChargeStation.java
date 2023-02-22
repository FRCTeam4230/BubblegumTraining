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

public class DrivePastChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation. */
  private final DriveTrainSubsystem driveTrain;
  private final DoubleSupplier pitchSupplier;
  private boolean AtChargeStation;
  private boolean OffChargeStation;
  private final PIDController rotationPidController;
  // private final PIDController distancePidController;

  public DrivePastChargeStation(DriveTrainSubsystem driveTrain, DoubleSupplier pitchSupplier) {
    super();
    this.driveTrain = driveTrain;
    this.pitchSupplier = pitchSupplier;

    AtChargeStation = false;
    OffChargeStation = false;
    
    
    rotationPidController = new PIDController(
      Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI, Constants.DriveTrain.TURN_KD);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    rotationPidController.setSetpoint(0);
    rotationPidController.setTolerance(1.5);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //this is the output for rotation
    double rotation = rotationPidController.calculate(driveTrain.getHeading());

    //Uses PID controller with robot heading to make sure the robot is going straight


    driveTrain.arcadeDrive(-0.5, MathUtil.clamp(rotation, -0.2, 0.2));

    //Pitch is negative as robot drives up charge station
    //If the pitch is more negative than 15, then the robot is at least
    //partially on the charge station
    if(pitchSupplier.getAsDouble() < -Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH) {
      AtChargeStation = true;
    }

    //After the robot has reached the charge station, if the pitch becomes very positive
    //which means that it is driving off of the charge station
    if(AtChargeStation && pitchSupplier.getAsDouble() > Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH) {
      OffChargeStation = true;
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
    //Once the robot is more or less on level ground after getting off of the charge station, end command
    return OffChargeStation && pitchSupplier.getAsDouble() < 5;
  }
}