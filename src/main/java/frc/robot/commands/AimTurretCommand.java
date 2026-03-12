package frc.robot.commands;

import static edu.wpi.first.units.Units.Inch;

import java.util.Collections;
import java.util.Set;
import java.util.function.DoubleSupplier;
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
import frc.robot.Constants;
import frc.robot.commands.AprilTagCoordinates;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.turret.TurretConstants;

public class AimTurretCommand extends Command {
    private final Swerve swerve;
    private final PIDController pid;
    private final double maxOutput;
    private final Set<Subsystem> requirements;
    private Shooter shooter;
    private DoubleSupplier shooterForward;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;


    public AimTurretCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, Shooter shooter, DoubleSupplier ShooterForward) {
        this.swerve = swerve;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.maxOutput = TurretConstants.MAX_OUTPUT;
        this.pid.setTolerance(1.0);
        this.requirements = Collections.singleton((Subsystem) this.swerve);
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.shooter = shooter;
        this.shooterForward = ShooterForward;
        addRequirements(shooter);
    }

@Override
public void initialize() {
    pid.reset();
}

@Override
public void execute() {
    ChassisSpeeds robotSpeeds = SwerveConfig.swerveKinematics.toChassisSpeeds(swerve.getModuleStates());
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getYaw());

    double dx = 4.611624 - swerve.getAprilOdom().getX();
    double dy = 4.021328 - swerve.getAprilOdom().getY();
    double targetdistance = Math.sqrt((dx * dx) + (dy * dy));

    double Vx = fieldSpeeds.vxMetersPerSecond;
    double Vy = fieldSpeeds.vyMetersPerSecond;

    double launchAngleDeg = 50;
    double launchHeight = 0.457;
    double hubHeight = 1.8288;
    double heightDiff = hubHeight - launchHeight;
    double g = 9.81;
    double flightTime = 1.0;
    double RPM = 0.0;
    double kMagnus = 0.0003; //Will need tuning

    double effectiveAngleRad = Math.toRadians(launchAngleDeg);

    if (targetdistance >= 2) {
        boolean converged = true;

        for (int angleIter = 0; angleIter < 5; angleIter++) {
            double t = 2.0;
            converged = true;

            for (int i = 0; i < 20; i++) {
                double tanA = Math.tan(effectiveAngleRad);
                double sinA = Math.sin(effectiveAngleRad);
                double f = Math.pow(heightDiff + 0.5 * g * t * t, 2) / Math.pow(tanA, 2)- Math.pow((dx - Vx * t), 2)- Math.pow((dy - Vy * t), 2);

                if (Math.abs(f) < 1e-3) {
                    flightTime = t;
                    break;
                }

                if (i == 19) {
                    System.out.println("Newton's method did not converge");
                    converged = false;
                    break;
                }

                double dfdt = 2 * g * t * (heightDiff + 0.5 * g * t * t) / Math.pow(tanA, 2)- 2 * Vx * (dx - Vx * t)- 2 * Vy * (dy - Vy * t);
                t = t - f / dfdt;
                if (t < 0) t = 4.0;
            }

            if (!converged) return;

            double launchSpeed = (heightDiff + 0.5 * g * flightTime * flightTime)
                               / (Math.sin(effectiveAngleRad) * flightTime);
            double distance_conv = 2.02395 * launchSpeed - 11.22416;
            RPM = -471.08803 * distance_conv - 2516.1188;

            double magnusCorrection = kMagnus * Math.abs(RPM);
            effectiveAngleRad = Math.toRadians(launchAngleDeg + magnusCorrection);
        }
    } else {
        System.out.println("Too close to find solution");
        return;
    }

    double launchSpeed = (heightDiff + 0.5 * g * flightTime * flightTime)/(Math.sin(effectiveAngleRad) * flightTime);
    double distance_conv = 2.02395 * launchSpeed - 11.22416;
    RPM = -471.08803 * distance_conv - 2516.1188;

    double TraverseAngle = Math.atan2(dy / flightTime - Vy, dx / flightTime - Vx);
    double currentHeadingRad = swerve.getYaw().getRadians();
    double errorDeg = Math.toDegrees(MathUtil.angleModulus(TraverseAngle - currentHeadingRad));
    double rotCMD = MathUtil.clamp(pid.calculate(0.0, errorDeg), -maxOutput, maxOutput);

    SmartDashboard.putNumber("errorDeg", errorDeg);
    SmartDashboard.putNumber("LaunchSpeed", launchSpeed);
    SmartDashboard.putNumber("FlightTime", flightTime);
    SmartDashboard.putNumber("TraverseAngle", Math.toDegrees(TraverseAngle));
    SmartDashboard.putNumber("Robot Yaw", swerve.getYaw().getDegrees());
    SmartDashboard.putNumber("RPM", RPM);
    SmartDashboard.putNumber("dx", dx);
    SmartDashboard.putNumber("dy", dy);
    SmartDashboard.putNumber("Vx", Vx);
    SmartDashboard.putNumber("Vy", Vy);
    SmartDashboard.putNumber("Total Dist", targetdistance);
    SmartDashboard.putNumber("EffectiveLaunchAngle", Math.toDegrees(effectiveAngleRad));
    SmartDashboard.putNumber("MagnusCorrection", kMagnus * Math.abs(RPM));

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    swerve.drive(new Translation2d(translationVal, strafeVal).times(3), rotCMD, true, true);

    if (shooterForward.getAsDouble() > 0.1) {
        shooter.setTargetRpm(RPM);
    }
}

@Override
public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0.0, true, true);
    shooter.setSpeed(0);
    pid.reset();
}

@Override
public boolean isFinished() {
    return false;
}

@Override
public Set<Subsystem> getRequirements() {
    return requirements;
}

@Override
public boolean runsWhenDisabled() {
    return false;
}
}
