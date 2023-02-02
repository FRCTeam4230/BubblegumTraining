// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class driveTrain {

    //Motor IDs
    public static final int LEFT_1_MOTOR_ID = 0;
    public static final int RIGHT_2_MOTOR_ID = 0;
    public static final int RIGHT_1_MOTOR_ID = 0;
    public static final int LEFT_2_MOTOR_ID = 0;

    //Multipliers
    public static final double SPEED_MULTIPLIER = 0.6;
    public static final double ROTATION_MULTIPLIER = 0.6;

    //Ramp rate
    public static final double DRIVE_RAMP_RATE = 0;

  }

  public static class arm {
    public static final int ARM_MOTOR_ID = 0;
    public static final double ARM_RAMP_RATE = 0;

  }

  public static class intake {

    public static final int INTAKE_MOTOR_ID = 0;
    public static final double INTAKE_RAMP_RATE = 0;
    public static final double INTAKE_SPEED = 0;

  }

  public static class OI {
    public static final int XBOX_PORT = 0;
  }
}
