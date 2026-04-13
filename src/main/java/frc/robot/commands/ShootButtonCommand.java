package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.ArmStuff.motor;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.commands.AprilTagCoordinates;


//While held, aim when in green zone, spin shooter to a looked-up target velocity based on distance, and run gumball when shooter is at speed. 
public class ShootButtonCommand extends Command {
    private final Swerve swerve;
    private final Shooter shooter;
    private final motor gumball;
    private final frc.robot.ShooterLookup lookup;
    private final PIDController pid;
    private final NetworkTableEntry txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final NetworkTableEntry tvEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv");
    private final Supplier<Translation2d> translationSupplier;
    private final double maxOutput;
    private double targetRpm = 200;
    private int stallBackwards = 0;
    private int counter = 0;



    private final double gumballSpeed = 0.2;

    public ShootButtonCommand(Swerve swerve, Shooter shooter, motor gumball, frc.robot.ShooterLookup lookup, Supplier<Translation2d> translationSupplier) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.gumball = gumball;
        this.lookup = lookup;
        this.translationSupplier = translationSupplier;
        this.pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        this.pid.setTolerance(1.0);
        this.maxOutput = TurretConstants.MAX_OUTPUT;
        addRequirements((Subsystem) swerve, (Subsystem) shooter, (Subsystem) gumball);
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
        double distToGoal = Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2));
        double TxGoal =  Math.toDegrees(-1*(Math.atan2(4.021328 - swerve.getAprilOdom().getY(), 4.611624 - swerve.getAprilOdom().getX())) + Math.atan2(AprilTagCoordinates.getY(26) - swerve.getAprilOdom().getY(), AprilTagCoordinates.getX(26) - swerve.getAprilOdom().getX())) + tx;
        double errorDeg = TxGoal - TurretConstants.CAMERA_TURRET_ANGLE_OFFSET_DEG;
        SmartDashboard.putNumber("TxGoal", TxGoal);
        SmartDashboard.putNumber("Tx", tx);
        System.out.println("robot position - x: " + swerve.getAprilOdom().getX() + " y: " + swerve.getAprilOdom().getY());
        System.out.println("limelight position - x: " + AprilTagCoordinates.getX(26) + " y: " + AprilTagCoordinates.getY(26));
        double rotCMD = pid.calculate(errorDeg, 0.0);
        rotCMD = MathUtil.clamp(rotCMD, -maxOutput, maxOutput);

        swerve.drive(translationSupplier.get().times(3), rotCMD, true, true);

        //double targetRpm = lookup.getSmoothedVelocity(distToGoal, 1.0);
        //if (Double.isNaN(targetRpm)) {
            targetRpm = SmartDashboard.getNumber("Manual Shooter RPM", 200);
        //    if (Double.isNaN(targetRpm)) {
        //        targetRpm = 0.0;
        //    }
        //}
        SmartDashboard.putNumber("Shooter Target RPM", targetRpm);

        shooter.setTargetRpm(targetRpm);

        double currentVel = shooter.getVelocity();
        SmartDashboard.putNumber("Shooter Current RPM", currentVel);
        boolean atSpeed = false;
        double tol = Math.max(100, Math.abs(targetRpm) * 0.02);
        atSpeed = Math.abs(currentVel - targetRpm) <= tol;

        if (atSpeed) {
            counter++;
            if(stallBackwards == 0) {
                gumball.setSpeed(gumballSpeed);
            }
            if (gumball.getIntakeCurrent() >= 2 && counter > 30){
                gumball.setSpeed(-gumballSpeed);
                stallBackwards++;
            }
            if (stallBackwards >= 1){
                counter = 0;
                stallBackwards++;
            }
            if (stallBackwards >= 20){
                stallBackwards = 0;
            }
        } else {
            gumball.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0.0);
        if(((gumball.getMotorPos()%4 <= .5))){
            gumball.setSpeed(0);
        }
        pid.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
