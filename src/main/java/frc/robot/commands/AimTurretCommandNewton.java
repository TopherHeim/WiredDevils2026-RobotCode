package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;

import java.util.Collections;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.AprilTagCoordinates;
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

    /* I don't know if I care if we have a target this simply use the robot yaw angle (field relative) 

    boolean hasTarget = tvEntry.getDouble(0.0) >= 1.0;
    if (!hasTarget) {
        swerve.drive(translationSupplier.get().times(3), 0.0, true, true);
        return;
    }
        */

    ChassisSpeeds robotSpeeds = SwerveConfig.swerveKinematics.toChassisSpeeds(swerve.getModuleStates());
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getYaw());

    // These will need to chagne based on Alliance for Blue vs. Red Hub.  Note that the math below is for y flipped vs. FRC coordinates.  Angle should be correct.
    double dx = 4.611624 - swerve.getAprilOdom().getX();
    double dy = swerve.getAprilOdom().getY() - 4.021328;

    double Vx = fieldSpeeds.vxMetersPerSecond;
    double Vy = -fieldSpeeds.vyMetersPerSecond;  // Note the negative sign to flip the y direction for the math below.  This is because the math is based on a coordinate system where positive y is up, but in FRC coordinates, positive y is forward.

    double launchAngleDeg = 47.5;
    double launchAngleRad = Math.toRadians(launchAngleDeg);
    double launchHeight = 0.457;   
    double hubHeight = 1.8288;    
    double heightDiff = hubHeight - launchHeight; 

    double g = 9.81;
    double cosA = Math.cos(launchAngleRad);
    double sinA = Math.sin(launchAngleRad);


// Solving for time using Newton's method.  

    for (int i = 1; i < 20; i++) {
        if (i == 1) { t = 2.0};

        double f = Math.pow(heightDiff + 0.5 * g * t * t, 2) / Math.pow(Math.tan(launchAngleRad),2) - Math.pow((dx - Vx * t),2) - Math.pow((dy - Vy * t),2);
         if (Math.abs(dfdt) < 1e-6) {
            double flightTime = t;
            break;
        }

        double dfdt = 2 * g * t * (heightDiff + 0.5 * g * t * t) / Math.pow(Math.tan(launchAngleRad),2) + 2 * Vx * (dx - Vx * t) + 2 * Vy * (dy - Vy * t);
        double t = t - f / dfdt;
        if (t < 0) {
            t = 4; // Prevent negative time
        }
        if (i == 19) {
            System.out.println("Newton's method did not converge");
        }
    }

    double launchSpeed = heightDiff + 0.5 * g * t * t / (sinA * t);
    double TraverseAngle = Math.atan2(dy/t - Vy, dx/t - Vx);
    

    double currentHeadingRad = swerve.getYaw().getRadians();
    double errorDeg = Math.toDegrees(MathUtil.angleModulus(TraverseAngle - currentHeadingRad)) - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;
    double rotCMD = MathUtil.clamp(pid.calculate(errorDeg, 0.0), -maxOutput, maxOutput);

    // Dashboard
    SmartDashboard.putNumber("LaunchSpeed", launchSpeed);
    SmartDashboard.putNumber("FlightTime", t);
    SmartDashboard.putNumber("TraverseAngle", Math.toDegrees(TraverseAngle));

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