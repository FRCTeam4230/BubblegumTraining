// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class driveTrain { //Classes always start with a capital. if you use the ide, you can rename this and then it'll rename it everywhere else for you

    //Multipliers
    public static final double SPEED_MULTIPLIER = 0.5;
    public static final double ROTATION_MULTIPLIER = 0.5;

    //Ramp rate
    public static final double DRIVE_RAMP_RATE = 1.0;
    public static final double  MOTOR_ROTATION_TO_INCHES = (1 / 42.0) * (1 / 8.45) * (6 * Math.PI);

    //PID
    public static double kStabilizationP = 0.04;
    public static double kStabilizationI = 0.0;
    public static double kStabilizationD = 0.0;


    public static final double CHARGING_STATION_SPEED_MULTIPLIER =  0;
    public static final double kPositionTolerance = 0.5;
    public static final double kVelocityTolerance = 0;
  }

  public static class arm {
    public static final int ARM_MOTOR_ID = 4;
    public static final double ARM_RAMP_RATE = 0.5;
    //Might change encoder to more accurate. Update the number 42 is we get new encoder
    public static final double MOTOR_TO_DEGREES = (1 / 42.0) * (1 / 48.0) * 360;
	public static final int ENCODER_PORT = 0;
	public static final int FRONT_LIMIT_PORT = 1;
	public static final int BACK_LIMIT_PORT = 2;
	public static final double FORWARD_LIMIT_ANGLE = 0;
	public static final double BACK_LIMIT_ANGLE = 0;
  


  }

  public static class intake {
    public static final double INTAKE_RAMP_RATE = 0.5;
    public static final double INTAKE_SPEED = 0.5; //50 % for now

  }

  public static class OI {
    public static final int XBOX_PORT = 0;
  }

  public static class ArmPIDConstants {

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double VELOCITY_TOLERANCE = 0;
    public static final double POSITION_TOLERANCE = 0;

  }

  public static class ArmPositions {

    //When the arm is inside the robot
    public static final double BRING_IN = 0;
    //When the robot scores in middle row
    public static final double SCORE = 0;
    //When the robot is picking stuff off of the ground
    public static final double PICK_UP_FROM_GROUND = 0;
    //When the robot is picking stuff from the loading station
    public static final double PICK_UP_FROM_STATION = 0;

  }

  public enum MotorID {
    LEFT_1_MOTOR_ID(1, driveTrain.DRIVE_RAMP_RATE), LEFT_2_MOTOR_ID(2, driveTrain.DRIVE_RAMP_RATE), RIGHT_1_MOTOR_ID(3, driveTrain.DRIVE_RAMP_RATE),
    RIGHT_2_MOTOR_ID(4, driveTrain.DRIVE_RAMP_RATE), ARM_MOTOR_ID(5, arm.ARM_RAMP_RATE), INTAKE_MOTOR_ID(6, intake.INTAKE_RAMP_RATE);

    private Integer id;
    private Double rampRate;

    private MotorID(Integer id, Double rampRate) {
      this.id = id;
    }

    public Integer getId() {
      return id;
    }
    
    public Double getRampRate(){
      return rampRate;
    }
  }

}
