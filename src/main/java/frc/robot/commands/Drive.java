// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  private final DriveTrainSubsystem driveTrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;
  private final DoubleSupplier armAngleSupplier;


  public Drive(DriveTrainSubsystem driveTrain, DoubleSupplier speed,  DoubleSupplier rotation, DoubleSupplier armAngleSupplier) {
    this.driveTrain = driveTrain;
    this.speed = speed;
    this.rotation = rotation;
    this.armAngleSupplier = armAngleSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double wantedSpeed = speed.getAsDouble();
    double wantedRotation = rotation.getAsDouble();

    double armAngle = armAngleSupplier.getAsDouble();
    boolean armOut = armAngle > 250;


    //Global motor limits
    if (armOut){
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_ARM_OUT_MAXIMUM;
      wantedRotation = wantedRotation * Constants.DriveTrain.ROTATION_ARM_OUT_MAXIMUM;
    }else{
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.ROTATION_MULTIPLIER;
    }

    //Process the speed again based off of arm angles
    if (armAngle > Constants.DriveTrain.ARM_OUT_BOUNDARY) {
      //If the arm is in the front
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_ARM_OUT_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.ROTATION_ARM_OUT_MULTIPLIER;
    } else if(armAngle < Constants.DriveTrain.ARM_IN_BOUNDARY) {
      //If the arm is inside the robot
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_ARM_IN_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.ROTATION_ARM_IN_MULTIPLIER;
    } else {
      //If the arm if up
      wantedSpeed = wantedSpeed * Constants.DriveTrain.SPEED_ARM_UP_MULTIPLIER;
      wantedRotation = wantedRotation * Constants.DriveTrain.ROTATION_ARM_UP_MULTIPLIER;
    }

    //now pass in the calcualted speed and rotation
    driveTrain.arcadeDrive(-wantedSpeed, wantedRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
