package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStuff.Shooter;

public class WaitForObjectToBeGrabbed extends Command {
    private Shooter intake;
    private double currentThreshold;  // The threshold for the current when an object is grabbed
    private double s = 0.15;  // The speed of the intake motor
    int count = 0;
    int count1 = 0;
    int count2 = 0;
    int count3 = 0;

    public WaitForObjectToBeGrabbed(Shooter intake, double currentThreshold) {
        this.intake = intake;
        this.currentThreshold = currentThreshold;
        addRequirements(intake);  // Make sure to require the intake subsystem
    }

    @Override
    public void initialize() {
        intake.setSpeed(s);  // Start the intake motor
    }

    @Override
    public void execute() {
        count2++;
        count3++;
    }

    @Override
    public boolean isFinished() {
        // Finish the command if the current exceeds the threshold (indicating the object is grabbed)
        if(intake.getCurrent() > currentThreshold && count2 >= 20){
            count++;
        }
        if(count >= 1){
            count1++;
        }
        if(count1 >= 15){
            count = 0;
            count1 = 0;
            count2 = 0;
            count3 = 0;
            intake.setSpeed(0.0);
            return true;
        }
        if(count3 >= 250){
            count = 0;
            count1 = 0;
            count2 = 0;
            count3 = 0;
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);  // Ensure the intake stops when the command ends
    }
}
