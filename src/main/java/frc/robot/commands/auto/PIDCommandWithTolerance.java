// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PIDCommandWithTolerance extends PIDCommand {
  /** Creates a new PIDCommandWithTolerance. */
  private SubsystemBase subsystemBase;

  public PIDCommandWithTolerance(PIDController pidController, DoubleSupplier measurementSource,
     DoubleSupplier setPointSource, DoubleConsumer useOutput, SubsystemBase subsystem, 
    double positionTolerance) {

    super(pidController, measurementSource, setPointSource, useOutput, subsystem);
    //getController().setTolerance(positionTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
      this.subsystemBase = subsystem;
    SmartDashboard.putData(getController());

    getController().setTolerance(positionTolerance);
    
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //this should be the driveTrain.isLevel but you can't do that from what y
    return ((DriveTrainSubsystem)subsystemBase).isLevel();
  }
}
