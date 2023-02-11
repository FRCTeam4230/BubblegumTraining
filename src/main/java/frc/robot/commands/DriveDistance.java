// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveDistanceParams;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private final DriveTrain driveTrain;

  private final PIDController mg1PidController = new PIDController(DriveDistanceParams.kP, DriveDistanceParams.kI,
      DriveDistanceParams.kD);
  private final PIDController mg2PidController = new PIDController(DriveDistanceParams.kP, DriveDistanceParams.kI,
      DriveDistanceParams.kD);

  private Double distance;

  public Double getDistance() {
    return distance;
  }

  private Double baseSpeed = DriveDistanceParams.baseSpeed;

  private DriveDistance(DriveTrain driveTrain) {
    this(driveTrain, 0);
  }

  /** Creates a new DriveDistance. */
  private DriveDistance(DriveTrain driveTrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);

    mg1PidController.setTolerance(DriveDistanceParams.tolerance, DriveDistanceParams.velocityTolerance);
    mg2PidController.setTolerance(DriveDistanceParams.tolerance, DriveDistanceParams.velocityTolerance);

    this.distance = distance;

    SmartDashboard.putData(this);
  }

  public static final DriveDistance create(DriveTrain driveTrain) {
    return new DriveDistance(driveTrain);
  }

  public static final DriveDistance create(DriveTrain driveSubsystem, Double distanceAsInches) {
    return new DriveDistance(driveSubsystem, distanceAsInches);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mg1PidController.setSetpoint(driveTrain.getLeftEncoder() + distance);
    mg2PidController.setSetpoint(driveTrain.getRightEncoder() + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mg1Output = mg1PidController.calculate(driveTrain.getLeftEncoder());
    double mg2Output = mg2PidController.calculate(driveTrain.getRightEncoder());

    mg1Output += Math.copySign(baseSpeed, mg1Output);
    mg2Output += Math.copySign(baseSpeed, mg2Output);

    mg1Output = MathUtil.clamp(mg1Output, -.5, .5);
    mg2Output = MathUtil.clamp(mg2Output, -.5, .5);

    driveTrain.setSpeeds(mg1Output, mg2Output);
    NetworkTableInstance.getDefault().getEntry("mg1Output").setDouble(mg1Output);
    NetworkTableInstance.getDefault().getEntry("mg2Output").setDouble(mg2Output);

    NetworkTableInstance.getDefault().getEntry("1at setpoint").setBoolean(mg1PidController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mg1PidController.atSetpoint() || mg2PidController.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("kP", mg1PidController::getP, kP -> {
      mg1PidController.setP(kP);
      mg2PidController.setP(kP);
    });

    builder.addDoubleProperty("kD", mg1PidController::getD, kD -> {
      mg1PidController.setD(kD);
      mg2PidController.setD(kD);
    });

    builder.addDoubleProperty("kI", mg1PidController::getI, kI -> {
      mg1PidController.setI(kI);
      mg2PidController.setI(kI);
    });

    builder.addDoubleProperty("base speed", () -> baseSpeed, s -> baseSpeed = s);
    builder.addDoubleProperty("distance", () -> distance, s -> distance = s);
  }
}
