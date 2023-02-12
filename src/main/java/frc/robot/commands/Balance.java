// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Balance extends PIDCommand {
  private DriveTrainSubsystem driveTrain;
  /** Creates a new Balance. */
  public Balance(DriveTrainSubsystem driveTrain) {
    super(
      new PIDController(
        Constants.DriveTrain.CHARGE_STATION_P,
        Constants.DriveTrain.CHARGE_STATION_I,
        Constants.DriveTrain.CHARGE_STATION_D),
  //Passes in measurement supplier
    driveTrain::getLeveledPitch,
  // Passes in setpoint
  driveTrain::getSetPoint,
  // Pipe the output to the turning controls
  output -> driveTrain.arcadeDrive(MathUtil.clamp(-output , -Constants.DriveTrain.PID_CLAMP_RANGE, Constants.DriveTrain.PID_CLAMP_RANGE), 0),
  // Require the robot drive 
  driveTrain);

  getController().setTolerance(Constants.DriveTrain.kPositionTolerance);

  this.driveTrain = driveTrain;
  this.driveTrain.lock();
  SmartDashboard.putData(this);
  }

  @Override
  public void execute() {
      // TODO Auto-generated method stub
      super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Bala3nce is finaished "+driveTrain.isLevel());
    return driveTrain.isLevel();
  }
}
