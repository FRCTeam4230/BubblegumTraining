package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;


public class weakBalance extends CommandBase {
  private final DriveTrainSubsystem driveTrain;
  private final PIDController drivePidController, turnPidController;

  public weakBalance(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    drivePidController = new PIDController(Constants.DriveTrain.CHARGE_STATION_P_WEAK, Constants.DriveTrain.CHARGE_STATION_I
            , Constants.DriveTrain.CHARGE_STATION_D);
    turnPidController = new PIDController(Constants.DriveTrain.TURN_KP, 0, 0);
    addRequirements(this.driveTrain);
  }

  @Override
  public void initialize() {
    drivePidController.setSetpoint(1.0);
    turnPidController.setSetpoint(driveTrain.getHeading());

    drivePidController.setTolerance(Constants.DriveTrain.POSITION_TOLERANCE);
    turnPidController.setTolerance(Constants.DriveTrain.TURN_TOLERANCE);
  }

  @Override
  public void execute() {
    double driveOutput = drivePidController.calculate(driveTrain.getPitch());
    double turnOutput = turnPidController.calculate(driveTrain.getHeading());

    driveOutput = MathUtil.clamp(driveOutput, -Constants.DriveTrain.PID_CLAMP_RANGE, Constants.DriveTrain.PID_CLAMP_RANGE);
    turnOutput = MathUtil.clamp(turnOutput, -Constants.DriveTrain.TURN_CLAMP_RANGE, Constants.DriveTrain.TURN_CLAMP_RANGE);

    driveTrain.arcadeDrive(driveOutput, turnOutput);

  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {

  }
}
