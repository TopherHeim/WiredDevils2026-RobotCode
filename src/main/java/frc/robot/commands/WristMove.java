package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class WristMove extends Command{
    private Wrist wrist;
    private BooleanSupplier run;
    private BooleanSupplier backwards;

    public WristMove(Wrist wrist, BooleanSupplier run, BooleanSupplier backwards){
        this.wrist = wrist;
        addRequirements(wrist);
        this.run = run;
        this.backwards = backwards;
    }

        @Override
    public void initialize(){
    }

        @Override
    public void execute(){
        SmartDashboard.putNumber("wirst enc", wrist.getPosition());
        if(run.getAsBoolean()){
            wrist.wristSetPoints(6.0);
        }
        else if(backwards.getAsBoolean()){
            wrist.wristSetPoints(0);
        }
    }
 
}
