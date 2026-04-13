package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * RecordSampleCommand records a single sample into ShooterLookup (in-memory only).
 * It reads a manual RPM from SmartDashboard (if present) or uses the shooter's encoder velocity.
 */
public class RecordSampleCommand extends InstantCommand {
    private final frc.robot.ShooterLookup lookup;
    private final Shooter shooter;
    private final Swerve swerve;

    public RecordSampleCommand(frc.robot.ShooterLookup lookup, Shooter shooter, Swerve swerve) {
        this.lookup = lookup;
        this.shooter = shooter;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        double manual = SmartDashboard.getNumber("Manual Shooter RPM", Double.NaN);
        double vel = Double.isNaN(manual) ? shooter.getVelocity() : manual;

        double dx = (swerve.getAprilOdom().getX() - 4.611624);
        double dy = (swerve.getAprilOdom().getY() - 4.021328);
        double distToGoal = Math.sqrt(dx * dx + dy * dy);

        lookup.addSample(distToGoal, vel);
        SmartDashboard.putNumber("ShooterLookup Count", lookup.getEntryCount());
    }
}
