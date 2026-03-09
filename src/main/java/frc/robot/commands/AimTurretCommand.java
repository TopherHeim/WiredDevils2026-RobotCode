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
import frc.robot.ShooterLookup;
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
    private final ShooterLookup lookup;
    private static final double HUB_X         = 4.611624;
    private static final double HUB_Y         = 4.021328;
    private static final double LAUNCH_ANGLE  = 47.5;
    private static final double LAUNCH_HEIGHT = 0.457;
    private static final double HUB_HEIGHT    = 1.8288;
    private static final double G             = 9.81;
    private final double sinA = Math.sin(Math.toRadians(LAUNCH_ANGLE));
    private final double cosA = Math.cos(Math.toRadians(LAUNCH_ANGLE));

    public AimTurretCommand(Swerve swerve, Supplier<Translation2d> translationSupplier, ShooterLookup lookup) {
        this.swerve = swerve;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.maxOutput = TurretConstants.MAX_OUTPUT;
        this.pid.setTolerance(1.0);
        this.requirments = Collections.singleton((Subsystem) this.swerve);
        this.translationSupplier = translationSupplier;
        this.lookup = lookup;
    }

    private double computeFlightTime(double launchSpeed, double distToGoal, double Vr) {
        double heightDiff = HUB_HEIGHT - LAUNCH_HEIGHT;
        double vVertical = launchSpeed * sinA;
        double discriminant = vVertical * vVertical - 2.0 * G * heightDiff;

        if (discriminant < 0.0) {
            return distToGoal / (launchSpeed * cosA + Vr);
        }

        return (vVertical - Math.sqrt(discriminant)) / G;
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        ChassisSpeeds robotSpeeds = SwerveConfig.swerveKinematics.toChassisSpeeds(swerve.getModuleStates());
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, swerve.getYaw());
        double Vx = fieldSpeeds.vxMetersPerSecond;
        double Vy = fieldSpeeds.vyMetersPerSecond;

        double dx = HUB_X - swerve.getAprilOdom().getX();
        double dy = HUB_Y - swerve.getAprilOdom().getY();
        double distToGoal = Math.hypot(dx, dy);
        double ux = dx / distToGoal;
        double uy = dy / distToGoal;

        double Vr = Vx * ux + Vy * uy;   
        double Vt = -Vx * uy + Vy * ux;  

        double roughRpm = lookup.getInterpolatedVelocity(distToGoal);
        double roughLaunchSpeed = Double.isNaN(roughRpm) ? 7.0 : Math.abs(roughRpm) / 600.0;
        double flightTime = computeFlightTime(roughLaunchSpeed, distToGoal, Vr);

        double adjustedDist = distToGoal + Vr * flightTime;
        adjustedDist = Math.max(0.1, adjustedDist);

        double targetRpm = lookup.getInterpolatedVelocity(adjustedDist);
        if (Double.isNaN(targetRpm)) {
            targetRpm = SmartDashboard.getNumber("Shooter Target Velocity", 0);
        }

        double GoalPositionCorrX = -uy * Vt * flightTime;
        double GoalPositionCorrY =  ux * Vt * flightTime;

        double desiredHeadingRad = Math.atan2((HUB_Y + GoalPositionCorrY) - swerve.getAprilOdom().getY(),(HUB_X + GoalPositionCorrX) - swerve.getAprilOdom().getX());
        double errorDeg = Math.toDegrees(MathUtil.angleModulus(desiredHeadingRad - swerve.getYaw().getRadians())) - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;
        double rotCMD = MathUtil.clamp(pid.calculate(errorDeg, 0.0), -maxOutput, maxOutput);

        SmartDashboard.putNumber("DistToGoal", distToGoal);
        SmartDashboard.putNumber("AdjustedDist", adjustedDist);
        SmartDashboard.putNumber("FlightTime", flightTime);
        SmartDashboard.putNumber("TargetRPM", targetRpm);
        SmartDashboard.putNumber("Vr", Vr);
        SmartDashboard.putNumber("Vt", Vt);
        SmartDashboard.putNumber("HeadingError", errorDeg);

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