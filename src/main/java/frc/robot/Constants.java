// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //THIS IS THE MOTOR GROUP CONTROL FLAG>
  //PROTOBOT is normal, 2023 ahs INVERTED gearboxes
  public final static boolean isProtoBot = false;

  public static class AutoConstants {

    //Change robot length to the length of charged up bot
    public static final double  DISTANCE_TO_CHARGE_STATION = 40;
    public static final double CHARGE_STATION_ONTO_PITCH = 15;

    public static final double PID_TURN_P = 0.0118;
    public static final double PID_TURN_I = 0;
    public static final double PID_TURN_D = 0;

  }


  public static final class DriveDistanceParams {
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double tolerance = 0.2;
    public static final double velocityTolerance = 0.2;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrain { 

    // Multipliers
    public static final double SPEED_MULTIPLIER = 0.8;
    public static final double ROTATION_MULTIPLIER = 0.7;
    
  
    // Ramp rate
    public static final double DRIVE_RAMP_RATE = 0.2;

    public static final double MOTOR_ROTATION_TO_INCHES = 72 / 40.687;//.03022; //pulse per



    // PID
    public static double CHARGE_STATION_P = 0.04;
    public static double CHARGE_STATION_I = 0.0;
    public static double CHARGE_STATION_D = 0.0;

    public static final double CHARGING_STATION_SPEED_MULTIPLIER = 0;
    public static final double kPositionTolerance = 0.8;
    public static final double kVelocityTolerance = 0;
    public static final double PID_CLAMP_RANGE = 0.45;

    public static final double ROBOT_LENGTH = 32.0;
    public static final double ROBOT_WIDTH = 24.0;

    public static final double DRIVE_P = 1.0;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    //The angle past which the arm is considered in front of the robot
    public static final double ARM_OUT_BOUNDARY = 230;
    //The angle in front of which the arm is considered to be in the robot
    public static final double ARM_IN_BOUNDARY = 95;
    //If the arm is either in front of the robot nor inside the robot, the arm is up

    //Multipliers for when the arm is out
    public static final double SPEED_ARM_OUT_MULTIPLIER = 0.7;
    public static final double ROTATION_ARM_OUT_MULTIPLIER = 0.7;
    //Multipliers for when the arm is in
    public static final double SPEED_ARM_IN_MULTIPLIER = 1;
    public static final double ROTATION_ARM_IN_MULTIPLIER = 1;
    //Multipliers for when the arm is up
    public static final double SPEED_ARM_UP_MULTIPLIER = 0.7;
    public static final double ROTATION_ARM_UP_MULTIPLIER = 0.7;

    //Default multiplier, all other multipliers are applied after this one
    public static final double SPEED_ARM_OUT_MAXIMUM = 0.7;
    public static final double ROTATION_ARM_OUT_MAXIMUM = 0.6;

    public static final double TURN_KP = 0.08;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;

    public static final int LED_LIGHTS_PORT = 3;

  }

  public static class Arm {
    public static final int ARM_MOTOR_ID = 4;
    public static final double ARM_RAMP_RATE = 0.5;
    // Might change encoder to more accurate. Update the number 42 is we get new
    // encoder
    public static final double MOTOR_TO_DEGREES = (1 / 42.0) * (1 / 48.0) * 360;
    public static final int ENCODER_PORT = 0;
    public static final int FRONT_LIMIT_PORT = 1;
    public static final int BACK_LIMIT_PORT = 2;
    public static final double FORWARD_LIMIT_ANGLE = 279;
    public static final double BACK_LIMIT_ANGLE = 0.5;
    public static final double ARM_SPEED = 0.35;

    //Constants for arm zones
    public static final double BOUNDARY_FAST_MINIMUM = 5;
    public static final double BOUNDARY_FAST_MAXIMUM = 250;
    public static final double ENTERING_ROTATION_SAFETY_ZONE_LIMIT = 0.35;
    public static final double EXITING_ROTATION_SAFETY_ZONE_LIMIT = 0.75;
    

  }

  public static class Intake {
    public static final double INTAKE_RAMP_RATE = 0.5;
    public static final double INTAKE_SPEED = 0.5; // 50 % for now
    public static final double MOTOR_ROTATION_TO_INCHES = (1 / 42.0) * (1 / 48.0) * 360; //copied from drivetrain

  }

  public static class OI {
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int INTAKE_XBOX_PORT = 1;
  }

  public static class ArmPIDConstants {

    public static final double kP_WITH_GRAVITY = 0.005;
    public static final double kI_WITH_GRAVITY = 0;
    public static final double kD_WITH_GRAVITY = 0;
    
    public static final double kP_AGAINST_GRAVITY = 0.03;
    public static final double kI_AGAINST_GRAVITY = 0;
    public static final double kD_AGAINST_GRAVITY = 0;

    public static final double VELOCITY_TOLERANCE = 0;
    public static final double POSITION_TOLERANCE = 0.5;
    public static final double RANGE = 0.2;

  }

  public static class ArmPositions {

    // When the arm is inside the robot
    public static final double BRING_IN = 1.5;
    // When the robot scores in top row
    public static final double SCORE_TOP = 193;
    //When the robot scores in the middle row
    public static final double SOCRE_MIDDLE = 210;
    // When the robot is picking stuff off of the ground
    public static final double PICK_UP_FROM_GROUND = 284;
    // When the robot is picking stuff from the loading station
    public static final double PICK_UP_FROM_STATION = 196;

  }

  public static class LightNumbers {
    public static final double PURPLE = 0.91;
    public static final double YELLOW = 0.69;
  }

  public enum MotorID {
    LEFT_1_MOTOR_ID(1, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES), 
    LEFT_2_MOTOR_ID(2,  DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),

    RIGHT_1_MOTOR_ID(3, DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES),
    RIGHT_2_MOTOR_ID(4,  DriveTrain.DRIVE_RAMP_RATE, IdleMode.kCoast, DriveTrain.MOTOR_ROTATION_TO_INCHES), 

    ARM_MOTOR_ID(6, Arm.ARM_RAMP_RATE, IdleMode.kBrake, Arm.MOTOR_TO_DEGREES),
    INTAKE_MOTOR_ID(5,  Intake.INTAKE_RAMP_RATE, IdleMode.kCoast, Intake.MOTOR_ROTATION_TO_INCHES);

    private Integer id;
    private Double rampRate;
    private IdleMode idleMode;
    private Double positionConversionFactor;

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

    public Double getPositionConversionFactor(){
      return this.positionConversionFactor;
    }

    public IdleMode getIdleMode(){
      return idleMode;
    }
  }

}
