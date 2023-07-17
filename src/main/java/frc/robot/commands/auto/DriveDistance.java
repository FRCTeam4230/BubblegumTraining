// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveDistanceParams;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveTrainSubsystem driveTrain;

  private final PIDController DistancePidController = new PIDController(DriveDistanceParams.kP, DriveDistanceParams.kI,
      DriveDistanceParams.kD);
  private final PIDController turnPidController = new PIDController(Constants.DriveTrain.TURN_KP, Constants.DriveTrain.TURN_KI,
  Constants.DriveTrain.TURN_KD);

  private double distance;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveTrainSubsystem driveTrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.distance = distance;


    DistancePidController.setTolerance(DriveDistanceParams.tolerance);
    turnPidController.setTolerance(DriveDistanceParams.tolerance);


    addRequirements(driveTrain);

    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroHeading();
    driveTrain.resetEncoders();

    DistancePidController.setSetpoint(distance);
    turnPidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = DistancePidController.calculate(driveTrain.getAverageEncoder());
    double turn = turnPidController.calculate(driveTrain.getRawHeading());

    speed = MathUtil.clamp(speed, -0.6, 0.6);
    turn = MathUtil.clamp(turn, -0.3, 0.3);

    //This is supposed to be speed then turn, but when that happens it rotates in place
    //This was flipped to turn and speed, and then it starts to go straight
    driveTrain.arcadeDrive(turn, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DistancePidController.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Distance: ", () -> distance, null);
  }
}
