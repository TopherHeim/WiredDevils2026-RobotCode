package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Voltage;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;



public class Shooter extends SubsystemBase implements ArmConstants  {

    public SparkMax shooterMotor;
    public RelativeEncoder shooterEncoder;
    public SparkClosedLoopController closedLoop;
    public double shooterSpeed;
   // public SparkMax coralShooter2;
   
    private static final double kS = 0.36904;
    private static final double kV = 0.12823;
    private static final double kA = 0.013269;
    
    private final SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(kS, kV, kA);

    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Second), // ramp rate
                Units.Volts.of(11),                   // max voltage
                null
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> shooterMotor.setVoltage(volts),
                log -> {log.motor("shooter").voltage(Units.Volts.of(shooterMotor.getAppliedOutput()* RobotController.getBatteryVoltage())).angularVelocity(Units.RotationsPerSecond.of(shooterEncoder.getVelocity() / 60.0)).angularPosition(Units.Rotations.of(shooterEncoder.getPosition()));},
                this
            )
        );


    public Shooter() {

        shooterMotor = new SparkMax(shooterId1, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
        closedLoop = shooterMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
            .p(0.0003)
            .i(0.0)
            .d(0.0)
            .outputRange(-1.0, 1.0);

        shooterMotor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setSpeed(double s){
        shooterMotor.set(s);
       // coralShooter2.set(s * -1);
    }
    public void ConditionSpeed(double s) {
        shooterSpeed = s;
    }

    public double DesiredShooterSpeed() {
        return shooterSpeed;
    }

    // Set shooter target in RPM using closed-loop controller
    public void setTargetRpm(double rpm) {
        double velocityRPS = rpm/60.0;
        double arbFF = feedforward.calculate(velocityRPS);
        closedLoop.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFF);
    }

    public void stopShooter() {
        shooterMotor.stopMotor();
    }

    public double getVelocity() {
        return shooterEncoder.getVelocity();
    }

    public double getCurrent() {
        return shooterMotor.getOutputCurrent();
    }

    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

}
