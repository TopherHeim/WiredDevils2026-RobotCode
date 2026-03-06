package frc.robot.subsystems.ArmStuff;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;

public class Elevator extends SubsystemBase implements ArmConstants {
    
    public SparkMax elevator;
    public SparkMax wrist;
    public SparkClosedLoopController elevatorPID;
    public SparkClosedLoopController wristPID;
    public SparkMaxConfig elevatorConfig;
    public SparkMaxConfig wristConfig;
    public RelativeEncoder eleEnc;
    public RelativeEncoder wristEnc;


    public Elevator(){
        
        elevator = new SparkMax(elevatorId, MotorType.kBrushless);
        wrist = new SparkMax(200, MotorType.kBrushless);

        eleEnc = elevator.getEncoder();
        wristEnc = wrist.getEncoder();

        elevatorConfig = new SparkMaxConfig();
        wristConfig = new SparkMaxConfig();

        elevatorPID = elevator.getClosedLoopController();
        wristPID = wrist.getClosedLoopController();

        elevatorConfig.closedLoop
            .p(eP)
            .i(eI)
            .d(eD)
            .outputRange(-1, 1);

        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        wristConfig.closedLoop
            .p(wP)
            .i(wI)
            .d(wD)
            .outputRange(-1, 1);

        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void moveElevator(double s){
        
        elevator.set(s);
    
        //elevatorPID.setReference(s *(5600), ControlType.kVelocity);
    }



    public void moveWrist(double s){

        wrist.set(s/5);
        //wristPID.setReference(s *(5600), ControlType.kVelocity);
    }

    public void wristSetPoints(double s){
        wristPID.setReference(s, ControlType.kPosition);
    }

    public void elevatorSetPoints(double s){
        elevatorPID.setReference(s, ControlType.kPosition);
    }

    public double elevatorPos(){
        return eleEnc.getPosition();
    }

    public double wristPos(){
        return wristEnc.getPosition();
    }

    public double getElevatorCurrent() {
        return elevator.getOutputCurrent();
    }



}
