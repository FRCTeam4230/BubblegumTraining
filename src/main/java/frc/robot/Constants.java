// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax.IdleMode;

public final class Constants {
  //All distances are measured in inches unless otherwise stated

  public static class CompetitionRobot {
    public final static double length = 34;
    public final static double width = 28;
  }

  public static class AutoConstants {

    public static final double DISTANCE_TO_CHARGE_STATION = 45;
    public static final double CHARGE_STATION_ONTO_PITCH = 14;

    public static final double LEFT_START_TO_COMMUNITY_LINE = 136;
    public static final double LEFT_COMMUNITY_LINE_TO_CENTER_LINE = 134;

    public static final double RIGHT_START_TO_COMMUNITY_LINE = 69;
    public static final double RIGHT_COMMUNITY_LINE_TO_CENTER_LINE = 203;
  }


  public static final class DriveDistanceParams {
    public static final double DRIVE_DISTANCE_P = 0.5;
    public static final double DRIVE_DISTANCE_I = 0;
    public static final double DRIVE_DISTANCE_D = 0;
    public static final double TOLERANCE = 0.2;
  }

  public static class DriveTrain {

    // Multipliers
    public static final double SPEED_MULTIPLIER = 0.9;
    public static final double ROTATION_MULTIPLIER = 0.9;


    // Ramp rate
    public static final double DRIVE_RAMP_RATE = 0.2;

    public static final double MOTOR_ROTATION_TO_INCHES = 72 / 40.687;//.03022; //pulse per
    public static final double TURN_CLAMP_RANGE = .4;
    public static final double TURN_TOLERANCE = 0.1;


    // PID
    public static double CHARGE_STATION_P = 0.04;

    public static double CHARGE_STATION_P_WEAK = 0.017;
    public static double CHARGE_STATION_I = 0.00;
    public static double CHARGE_STATION_D = 0.005;

    public static final double POSITION_TOLERANCE = 0.8;
    //This was 0.4
    public static double PID_CLAMP_RANGE = 0.4;

    //The angle past which the arm is considered in front of the robot
    public static final double ARM_OUT_BOUNDARY = 230;
    //The angle in front of which the arm is considered to be in the robot
    public static final double ARM_IN_BOUNDARY = 95;
    //If the arm is either in front of the robot nor inside the robot, the arm is up

    //Multipliers for when the arm is out
    public static final double SPEED_ARM_OUT_MULTIPLIER = 0.8;
    public static final double ROTATION_ARM_OUT_MULTIPLIER = 0.7;
    //Multipliers for when the arm is in
    public static final double SPEED_ARM_IN_MULTIPLIER = 1;
    public static final double ROTATION_ARM_IN_MULTIPLIER = 1;
    //Multipliers for when the arm is up
    public static final double SPEED_ARM_UP_MULTIPLIER = 0.7;
    public static final double ROTATION_ARM_UP_MULTIPLIER = 0.7;

    //Default multiplier, all other multipliers are applied after this one
    public static final double SPEED_ARM_OUT_MAXIMUM = 0.9;
    public static final double ROTATION_ARM_OUT_MAXIMUM = 0.6;

    public static final double TURN_KP = -0.07;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;
  }

  public static class Arm {
    public static final double ARM_RAMP_RATE = 0.5;
    public static final double MOTOR_TO_DEGREES = 1;

    public static final int ENCODER_PORT = 0;
    public static final int FRONT_LIMIT_PORT = 1;
    public static final int BACK_LIMIT_PORT = 2;
    public static final double FORWARD_LIMIT_ANGLE = 279;
    public static final double BACK_LIMIT_ANGLE = 0.5;
    public static final double ARM_SPEED = 0.2;

    //Constants for arm zones
    public static final double BOUNDARY_FAST_MINIMUM = 5;
    public static final double BOUNDARY_FAST_MAXIMUM = 255;
    public static final double ENTERING_ROTATION_SAFETY_ZONE_LIMIT = 0.8;
    public static final double EXITING_ROTATION_SAFETY_ZONE_LIMIT = 0.85;


  }

  public static class Intake {
    public static final double INTAKE_RAMP_RATE = 0.4;
    public static final double INTAKE_SPEED = 0.75;
    public static final double MOTOR_ROTATION_TO_INCHES = (1 / 42.0) * (1 / 48.0) * 360;

  }

  public static class OI {
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int INTAKE_XBOX_PORT = 1;
  }

  public static class ArmPIDConstants {

    public static final double P_WITH_GRAVITY = 0.005;
    public static final double I_WITH_GRAVITY = 0;
    public static final double D_WITH_GRAVITY = 0;

    public static final double P_AGAINST_GRAVITY = 0.03;
    public static final double I_AGAINST_GRAVITY = 0;
    public static final double D_AGAINST_GRAVITY = 0;

    public static final double POSITION_TOLERANCE = 0.5;

  }

  public static class ArmPositions {

    // When the arm is inside the robot
    public static final double BRING_IN = 1.5;
    // When the robot scores in top row
    public static final double SCORE_TOP = 193;
    //When the robot scores in the middle row
    public static final double SCORE_MIDDLE = 210;
    // When the robot is picking stuff off of the ground
    public static final double PICK_UP_FROM_GROUND = 284;
    // When the robot is picking stuff from the loading station
    public static final double PICK_UP_FROM_STATION = 195;

  }

  public enum MotorID {
    LEFT_1_MOTOR_ID(1, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),
    LEFT_2_MOTOR_ID(2, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),

    RIGHT_1_MOTOR_ID(3, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),
    RIGHT_2_MOTOR_ID(4, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),

    ARM_MOTOR_ID(6, Arm.ARM_RAMP_RATE, IdleMode.kBrake, Arm.MOTOR_TO_DEGREES),
    INTAKE_MOTOR_ID(5, Intake.INTAKE_RAMP_RATE, IdleMode.kCoast, Intake.MOTOR_ROTATION_TO_INCHES);

    private final Integer id;
    private final Double rampRate;
    private final IdleMode idleMode;
    private final Double positionConversionFactor;

    private MotorID(Integer id, Double rampRate, IdleMode idleMode, Double positionConversionFactor) {
      this.id = id;
      this.rampRate = rampRate;
      this.idleMode = idleMode;
      this.positionConversionFactor = positionConversionFactor;
    }

    public Integer getId() {
      return id;
    }

    public Double getRampRate() {
      return rampRate;
    }

    public Double getPositionConversionFactor() {
      return this.positionConversionFactor;
    }

    public IdleMode getIdleMode() {
      return idleMode;
    }
  }

}
