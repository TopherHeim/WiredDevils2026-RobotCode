package frc.robot.subsystems.ArmStuff;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase implements ArmConstants {
    public SparkMax wrist;
    public RelativeEncoder wristEncoder;
    public SparkClosedLoopController closedLoop;
    public double wristSpeed;
   // public SparkMax coralShooter2;
   
    

    public Wrist(){
        wrist = new SparkMax(wristId, MotorType.kBrushless);
        wristEncoder = wrist.getEncoder();
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .p(wP)
            .i(wI)
            .d(wD)
            .outputRange(-0.25, 0.1);
        wrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoop = wrist.getClosedLoopController();

        //coralShooter2 = new SparkMax(shooterId2, MotorType.kBrushless);
    }

    public void setSpeed(double s){
        wrist.set(s);
       // coralShooter2.set(s * -1);
    }

    public void wristSetPoints(double s){
        closedLoop.setReference(s, ControlType.kPosition);
    }

    public double getPosition(){
        return wristEncoder.getPosition();
    }
}
