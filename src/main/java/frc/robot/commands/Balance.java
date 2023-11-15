package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;


public class Balance extends CommandBase {
  private final DriveTrainSubsystem driveTrain;
  private final PIDController drivePidController, turnPidController;

  private double outputTurn = 0.0;
  private double outputDrive = 0.0;

  public Balance(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    drivePidController = new PIDController(Constants.DriveTrain.CHARGE_STATION_P, Constants.DriveTrain.CHARGE_STATION_I
            , Constants.DriveTrain.CHARGE_STATION_D);
    turnPidController = new PIDController(Constants.DriveTrain.TURN_KP, 0, Constants.DriveTrain.TURN_KD);
    turnPidController.enableContinuousInput(-180, 180);
    addRequirements(this.driveTrain);

    SmartDashboard.putData(this);
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
    if (Math.abs(driveTrain.getPitch() - 2) < 5) {
      drivePidController.setP(Constants.DriveTrain.CHARGE_STATION_P_WEAK);
    } else {
      drivePidController.setP(Constants.DriveTrain.CHARGE_STATION_P);
    }

    double driveOutput = drivePidController.calculate(driveTrain.getPitch());
    double turnOutput = turnPidController.calculate(driveTrain.getHeading());

    driveOutput = MathUtil.clamp(driveOutput, -Constants.DriveTrain.PID_CLAMP_RANGE, Constants.DriveTrain.PID_CLAMP_RANGE);
    turnOutput = MathUtil.clamp(turnOutput, -Constants.DriveTrain.TURN_CLAMP_RANGE, Constants.DriveTrain.TURN_CLAMP_RANGE);

    outputDrive = driveOutput;
    outputTurn = turnOutput;

    driveTrain.arcadeDrive(driveOutput, turnOutput);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrain.lock();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("P", () -> Constants.DriveTrain.CHARGE_STATION_P, null);
    builder.addDoubleProperty("P weak", () -> Constants.DriveTrain.CHARGE_STATION_P_WEAK, null);
    builder.addDoubleProperty("I", () -> Constants.DriveTrain.CHARGE_STATION_I, null);
    builder.addDoubleProperty("D", () -> Constants.DriveTrain.CHARGE_STATION_D, null);
    builder.addDoubleProperty("turn", () -> outputTurn, null);
    builder.addDoubleProperty("drive", () -> outputDrive, null);
  }
}
