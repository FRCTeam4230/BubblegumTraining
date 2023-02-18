package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class HoldArmCommand extends ArmPIDWithGravity {

    public HoldArmCommand(ArmSubsystem armSubsystem, double targetAngle) {
        //The pid controller isn't being used right now
        super(armSubsystem, () -> targetAngle);
        getController().setTolerance(0);


        SmartDashboard.putData(this);
    }

    //Does this override do anything significant?
    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            armSubsystem.stop();
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }

    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);
    }


    public void execute(){
        //Holds it be moving the arm up a tiny bit
        armSubsystem.holdAgainstGravity();
    }
    
}
