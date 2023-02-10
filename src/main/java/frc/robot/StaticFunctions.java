package frc.robot;

import java.util.function.Function;

import frc.robot.Constants.MotorID;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class StaticFunctions {

      // Function for configuring spark maxes
  public static Function<MotorID, CANSparkMax> initiateCANSparkMaxMotor = (id) -> {// Lambda notation
    CANSparkMax motor = new CANSparkMax(id.getId(), MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setOpenLoopRampRate(id.getRampRate());
    motor.setIdleMode(IdleMode.kCoast);

    RelativeEncoder maxEncoder = motor.getEncoder();
    maxEncoder.setPositionConversionFactor(Constants.driveTrain.MOTOR_ROTATION_TO_INCHES);

    return motor;
  };
    
}
