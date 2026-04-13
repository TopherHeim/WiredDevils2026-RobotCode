package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.AimTurretCommand;

public class CoralShoot extends Command {
    private Swerve swerve;
    private Shooter shooter;
    private DoubleSupplier shooterForward;
    private BooleanSupplier turretAutoAim;
    private static final double HUB_X = 4.611624;
    private static final double HUB_Y = 4.021328;    

    public CoralShoot(Swerve s_Swerve, Shooter shooter, DoubleSupplier ShooterForward, BooleanSupplier turretAutoAim) {
        this.swerve = s_Swerve;
        this.shooter = shooter;
        this.shooterForward = ShooterForward;
        this.turretAutoAim = turretAutoAim;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){ 

        if(!turretAutoAim.getAsBoolean()){
            if (shooterForward.getAsDouble() > 0.1){
                shooter.setTargetRpm((-451.08803*Math.sqrt(Math.pow((swerve.getAprilOdom().getX() - HUB_X), 2) + Math.pow((swerve.getAprilOdom().getY() - HUB_Y), 2))) - 2516.1188);
            }
            else {
                shooter.setSpeed(0);
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