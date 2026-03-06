package frc.robot.subsystems.ArmStuff;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements ArmConstants{
    public SparkMax intake;
    public RelativeEncoder intakeEncoder;
    public double intakeSpeed;
   
    

    public Intake(){
        intake = new SparkMax(intakeId, MotorType.kBrushless);
    }

    public void setSpeed(double s){
        intake.set(s);
    }

}
