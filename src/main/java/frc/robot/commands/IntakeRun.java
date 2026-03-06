package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.ArmStuff.Intake;

public class IntakeRun extends Command{
    
    private Intake intake;
    private DoubleSupplier run;

    public IntakeRun(Intake intake, DoubleSupplier run){
        this.intake = intake;
        addRequirements(intake);
        this.run = run;
    }

        @Override
    public void execute(){
        
        if (run.getAsDouble() >= 0.075){
            intake.setSpeed(.4);
        }
        else {
            intake.setSpeed(0);
        }
    }

}
