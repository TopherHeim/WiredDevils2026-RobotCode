package frc.robot.commands;


import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.ArmStuff.motor;


public class motortest extends Command{


    private motor motor;
    private Shooter shooter;
    private BooleanSupplier motor1;
    private int counter = 0;
    private int stallBackwards = 0;
    private boolean ShooterRevedup;


    public motortest(motor motor, Shooter shooter, BooleanSupplier motor1) {
        this.motor = motor;
        this.shooter = shooter;
        this.motor1 = motor1;
        addRequirements(motor);
    }


    public void execute(){
        ShooterRevedup = true; //shooter.getVelocity() >= shooter.DesiredShooterSpeed() - 50;
        if (ShooterRevedup) {

            counter++;
            if (motor1.getAsBoolean() == true && stallBackwards == 0){
                motor.setSpeed(0.50);
            }
            if (motor.getIntakeCurrent() >= 25 && counter > 30){
                motor.setSpeed(-0.2);
                stallBackwards++;
            }
            if(stallBackwards >= 1){
                counter = 0;
                stallBackwards++;
            }
            if(stallBackwards >= 20){
                stallBackwards = 0;
            } 
     }

        SmartDashboard.putNumber("Gumball", motor.getMotorPos());
        SmartDashboard.putNumber("Gumballenc", motor.getMotorPos()%1440);
        SmartDashboard.putNumber("Gumball Current", motor.getIntakeCurrent());
        SmartDashboard.putNumber("gumball speed", motor.getVelocity());
        SmartDashboard.putNumber("DesiredShooterSPeed", shooter.DesiredShooterSpeed());
    }
   
     @Override
     public boolean isFinished() {
        if((((motor1.getAsBoolean() == false || !ShooterRevedup) && motor.getMotorPos()%4 <= .5))){
            counter = 0;
            return true;
        }
        else {
            return false;
        }
     }
     
     @Override
     public void end(boolean interrupted) {
         motor.setSpeed(0);
     }


}
