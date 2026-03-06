package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.turret.TurretConstants;

public class AimTurretCommand extends Command {
    private final Swerve swerve;
    private final PIDController pid;
    private final double maxOutput;
    private final Set<Subsystem> requirments;
    private final Supplier<Translation2d> translationSupplier;

    private final NetworkTableEntry txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final NetworkTableEntry tvEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");

    public AimTurretCommand(Swerve swerve, Supplier<Translation2d> translationSupplier) {
        this.swerve = swerve;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.maxOutput = TurretConstants.MAX_OUTPUT;
        this.pid.setTolerance(1.0);
        this.requirments = Collections.singleton((Subsystem) this.swerve);
        this.translationSupplier = translationSupplier;
    }

@Override
public void initialize() {
    pid.reset();
}

@Override
public void execute() {
    
    boolean hasTarget = tvEntry.getDouble(0.0) >= 1.0;
    if (!hasTarget){
        swerve.drive(translationSupplier.get().times(3), 0.0, true, true);
        return;
    }
    
    double tx = txEntry.getDouble(0.0);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tidEntry = table.getEntry("tid");
    double TxGoal =  Math.toDegrees(-1*(Math.atan2(4.021328 - swerve.getAprilOdom().getY(), 4.611624 - swerve.getAprilOdom().getX())) + Math.atan2(AprilTagCoordinates.getY((int) tidEntry.getDouble(26)) - swerve.getAprilOdom().getY(), AprilTagCoordinates.getX((int) tidEntry.getDouble(26)) - swerve.getAprilOdom().getX())) + tx;
    double errorDeg = TxGoal - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;
    // double TxGoal = tx - 15;
    SmartDashboard.putNumber("TxGoal", TxGoal);
    SmartDashboard.putNumber("Tx", tx);
    System.out.println("robot position - x: " + swerve.getAprilOdom().getX() + " y: " + swerve.getAprilOdom().getY());
    double rotCMD = pid.calculate(errorDeg, 0.0);
    rotCMD = MathUtil.clamp(rotCMD, -maxOutput, maxOutput);

    swerve.drive(translationSupplier.get().times(3), rotCMD, true, true);
}

@Override
public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0.0, true, true);
    pid.reset();
}

@Override
public boolean isFinished() {
    return false;
}

@Override
public Set<Subsystem> getRequirements() {
    return requirments;
}

@Override
public boolean runsWhenDisabled() {
    return false;
}
}
