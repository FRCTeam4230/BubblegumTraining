package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class HoldArmCommand extends ArmPID {

    public HoldArmCommand(ArmSubsystem armSubsystem, Double targetAngle) {
        super(armSubsystem, () -> targetAngle);
        getController().setTolerance(0);


        SmartDashboard.putData(this);
    }

    public boolean isFinished(){
        return false;
    }

    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);
    }


    public void execute(){
        armSubsystem.hold();
    }
    
}
