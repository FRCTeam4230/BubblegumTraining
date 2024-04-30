package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystem.DriveTrainSubsystem;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class DriveCommand implements Command {
    private final DriveTrainSubsystem driveTrainSubsystem;
    private final Set<Subsystem> subsystems;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier turnSupplier;

    public DriveCommand(DriveTrainSubsystem driveTrainSubsystem, DoubleSupplier speedSupplier, DoubleSupplier turnSupplier) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.subsystems = Set.of(this.driveTrainSubsystem);
        this.speedSupplier = speedSupplier;
        this.turnSupplier = turnSupplier;
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.arcadeDrive(speedSupplier.getAsDouble(), turnSupplier.getAsDouble());
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return this.subsystems;
    }
}
