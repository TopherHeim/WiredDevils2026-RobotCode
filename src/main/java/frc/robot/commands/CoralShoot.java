package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class CoralShoot extends Command {
    private Swerve swerve;
    private Shooter shooter;
    private DoubleSupplier shooterForward;
    

    public CoralShoot(Swerve s_Swerve, Shooter shooter, DoubleSupplier ShooterForward) {
        this.swerve = s_Swerve;
        this.shooter = shooter;
        this.shooterForward = ShooterForward;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        shooter.setTargetRpm(SmartDashboard.getNumber("Shooter Target Velocity", 0));
        SmartDashboard.putNumber("Shooter Rpm", shooter.getVelocity());
        SmartDashboard.putNumber("Shooter Current", shooter.getCurrent());
        if(shooterForward != null) {
        boolean GreenZone = (Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)) <= 3) && (Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)) >= .5);
        SmartDashboard.putBoolean("Green Zone", GreenZone);
        SmartDashboard.putNumber("Green Zone Number", Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - 4.611624), 2) + Math.pow((swerve.getAprilOdom().getY() - 4.021328), 2)));
        SmartDashboard.putNumber("Shooter get Velocity", shooter.getVelocity());
        if (GreenZone) {
            if (shooterForward.getAsDouble() > 0.1){
                shooter.setSpeed(-1*shooterForward.getAsDouble() * .85);
                //shooter.ConditionSpeed(shooterForward.getAsDouble() * 5000);
            }
            else {
                shooter.setSpeed(0);
                //shooter.ConditionSpeed(1000000);
            }  
        }
        else {
            shooter.setSpeed(0);
            //shooter.ConditionSpeed(10000000);
        }
    }

    }
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
	public void end(boolean interrupted){
		
	}    
}