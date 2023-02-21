// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveToChargeStation extends CommandBase {
  /** Creates a new DriveToChargeStation. */
  private final DriveTrainSubsystem driveTrain;
  private final DoubleSupplier pitchSupplier;
  private final DoubleSupplier distanceSupplier;
  //private boolean AtChargeStation;
  //private boolean OffChargeStation;
  //private double cyclesElapsed;
  //private double cyclesElapsed2;
  private final PIDController rotationPidController;
  private final PIDController distancePidController;

  public DriveToChargeStation(DriveTrainSubsystem driveTrain, DoubleSupplier pitchSupplier, DoubleSupplier distanceSupplier) {
    super();
    //PID CONTROLLER FOR DISTANC
    this.driveTrain = driveTrain;
    this.pitchSupplier = pitchSupplier;
    this.distanceSupplier = distanceSupplier;
    //i wouldn't do this.
    //AtChargeStation = false;
    //OffChargeStation = false;
    
    
    rotationPidController = new PIDController(
      Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI, Constants.DriveTrain.TURN_KD);
    
    distancePidController = new PIDController(Constants.DriveDistanceParams.kP, Constants.DriveDistanceParams.kI, Constants.DriveDistanceParams.kD);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    rotationPidController.setSetpoint(0);
    rotationPidController.setTolerance(1.5);

    distancePidController.setSetpoint(distanceSupplier.getAsDouble());
    distancePidController.setTolerance(1); // 1inch

    //cyclesElapsed = 0;
    //cyclesElapsed2 = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //this is the output for rotation
    double rotation = rotationPidController.calculate(driveTrain.getHeading());

    //user ANOTHER pID CONTROLLER FOR DISTNANCE
    double speed = distancePidController.calculate(driveTrain.getAverageEncoder());

    driveTrain.arcadeDrive(MathUtil.clamp(speed, -0.4, 0.4), MathUtil.clamp(rotation, -0.2, 0.2));

    // if(cyclesElapsed > 10 && cyclesElapsed < 15) {
    //   driveTrain.arcadeDrive(0, MathUtil.clamp(output, -0.2, 0.2));
    // } else if(cyclesElapsed > 15) {
    //   driveTrain.arcadeDrive(-0.7, MathUtil.clamp(output, -0.2, 0.2));
    // } else {
      // driveTrain.arcadeDrive(-0.6, MathUtil.clamp(output, -0.2, 0.2));
    // }

    //Drive at half speed

    //Uses PID controller with robot heading to make sure the robot is going straight

    
/* 
    if(OffChargeStation) {
      driveTrain.arcadeDrive(0.4, MathUtil.clamp(output, -0.2, 0.2));
    } else {
      driveTrain.arcadeDrive(-0.6, MathUtil.clamp(output, -0.2, 0.2));

    }
    */

    // if(AtChargeStation) {
    //   //If the robot is at the charge station, add 1 to cyclesElapsed
    //   //cyclesElasped is used in the isFinished method to determine when the command should end
    //   cyclesElapsed += 1;
    // }

    /*
    if(pitchSupplier.getAsDouble() > Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH) {
      //If the pitch goes past 15 degrees, we know that the robot is at least partially on the charge station
      AtChargeStation = true;
    }

    if(AtChargeStation && pitchSupplier.getAsDouble() < 5) {
      OffChargeStation = true;
      cyclesElapsed2 += 1;
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return pitchSupplier.getAsDouble() > Constants.AutoConstants.CHARGE_STATION_ONTO_PITCH || distancePidController.atSetpoint();
  }
}
