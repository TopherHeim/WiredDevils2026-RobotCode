package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.AutonCommands.ElevatorHeight;
import frc.robot.commands.AutonCommands.Shoot;
import frc.robot.commands.AutonCommands.WaitForObjectToBeGrabbed;
import frc.robot.commands.AutonCommands.WristHeight;
import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.ArmStuff.Actuator;
import frc.robot.subsystems.ArmStuff.Actuator2;
import frc.robot.subsystems.ArmStuff.Climber;
import frc.robot.subsystems.ArmStuff.Elevator;
import frc.robot.subsystems.ArmStuff.Shooter;
import frc.robot.subsystems.ArmStuff.motor;
import frc.robot.ShooterLookup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.commands.ShootButtonCommand;
import frc.robot.commands.RecordSampleCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    /* Swerve SubSystem */
    public Swerve getSwerveSubsystem(){
        return s_Swerve;
    }
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick upper = new Joystick(1);


   /* Driver Controls */
	private final int translationAxis = XboxController.Axis.kLeftY.value;
	private final int strafeAxis = XboxController.Axis.kLeftX.value;
	private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Upper Controls */
    private final int wristMove = XboxController.Axis.kRightY.value;
    //private final int pickUp = XboxController.Axis.kLeftY.value;
    private final int elevatorMove = XboxController.Axis.kLeftY.value;
    private final int climbUp = XboxController.Axis.kRightTrigger.value;
    private final int climbDown = XboxController.Axis.kLeftTrigger.value;
    //private final int climbUp = XboxController.Axis.kRightY.value;
    private final int actuator2Move = XboxController.Axis.kLeftY.value;

    /* Upper Buttons */
    //private final JoystickButton shooterSpeedOne = new JoystickButton(upper, XboxController.Button.kA.value);
    //private final JoystickButton retractActuator = new JoystickButton(upper, XboxController.Button.kB.value);
    private final JoystickButton elevatorSetPoint1 = new JoystickButton(upper, XboxController.Button.kB.value);
    private final JoystickButton shooterBackward = new JoystickButton(upper, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shooterForward = new JoystickButton(upper, XboxController.Button.kRightBumper.value);
    private final JoystickButton wristSetPoint1 = new JoystickButton(upper, XboxController.Button.kY.value);
    private final JoystickButton wristSetPoint2 = new JoystickButton(upper, XboxController.Button.kX.value);
    private final JoystickButton motorinput = new JoystickButton(upper, XboxController.Button.kA.value);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton extendActuator = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton retractActuator = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton dampen = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton DriveToApril = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton turretAutoAimButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter shooter = new Shooter();
    private final motor motor = new motor();
    // Lookup table instance to map distance -> shooter velocity/percent
    private final ShooterLookup shooterLookup = new ShooterLookup();
    private final Actuator2 actuator2 = new Actuator2();
    public final Elevator elevator = new Elevator();
    private final Actuator actuator = new Actuator();
    private final Climber climb = new Climber();
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    


    //private PathPlannerTrajectory trajectory;
    //private final Command auton = new AutonRun(s_Swerve);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Register Named Commands
        registerNamedCommands();
        
        turretAutoAimButton.whileTrue(
            new AimTurretCommand(
                s_Swerve,
                () -> new Translation2d(-driver.getRawAxis(translationAxis), -driver.getRawAxis(strafeAxis)),
                shooterLookup
            )
        );

        motorinput.whileTrue(
            new ShootButtonCommand(
                s_Swerve,
                shooter,
                motor,
                shooterLookup,
                () -> new Translation2d(-driver.getRawAxis(translationAxis), -driver.getRawAxis(strafeAxis))
            )
        );

        SmartDashboard.putNumber("Manual Shooter RPM", Double.NaN);
        SmartDashboard.putNumber("ShooterLookup Count", shooterLookup.getEntryCount());
        SmartDashboard.putData("Record Shooter Sample", new RecordSampleCommand(shooterLookup, shooter, s_Swerve));
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -Math.pow(driver.getRawAxis(translationAxis),3), 
                () -> -Math.pow(driver.getRawAxis(strafeAxis),3), 
                () -> -Math.pow(driver.getRawAxis(rotationAxis),3), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 1, //speed multiplier 
                () -> DriveToApril.getAsBoolean()
		
            )
        );
    /* */    
        elevator.setDefaultCommand(
            new ElevatorRun(
                elevator,
                () -> upper.getRawAxis(elevatorMove),
                () -> upper.getRawAxis(wristMove),
                () -> wristSetPoint1.getAsBoolean(),
                () -> elevatorSetPoint1.getAsBoolean(),
                () -> wristSetPoint2.getAsBoolean()
            
            )
        ); 
        

        

        // Comment these out when testing drive// 
        
        actuator2.setDefaultCommand(
            new Actuator2Run(
                actuator2,
                () -> extendActuator.getAsBoolean(),
                () -> retractActuator.getAsBoolean()
            )
        );
    
        shooter.setDefaultCommand(
            new CoralShoot(
                s_Swerve,
                shooter,
                () -> shooterForward.getAsBoolean(), 
                () -> shooterBackward.getAsBoolean()
            )
        );

 
        motor.setDefaultCommand(
            new motortest(
                motor,
                () -> motorinput.getAsBoolean()
            )
        );
        
        /* 
        pickUpper.setDefaultCommand(
            new PickerUp(
                pickUpper,
                () -> upper.getRawAxis(movePickUpArm),
                () -> upper.getRawAxis(pickUp),
                () -> pickUpFloorPoint.getAsBoolean(),
                () -> fastMode.getAsBoolean()
            )
        );
        
        actuator.setDefaultCommand(
            new ActuatorRun(
                actuator,
                () -> upper.getRawAxis(shooterAim),
                () -> actuatorZero.getAsBoolean(),
                () -> actuatorShoot.getAsBoolean()
            )
        );
         */
        
        climb.setDefaultCommand(
            new Climb(
                climb,
                () -> driver.getRawAxis(climbUp),
                () -> driver.getRawAxis(climbDown)
            )
        );
    
        /*
        s_Swerve.setDefaultCommand(
            new ZeroWheels(
                s_Swerve,
                () -> zeroWheels.getAsBoolean()
            )
        );
        */
        
        // Configure the button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }


    public void registerNamedCommands(){
        
        /////////////////////////////////////Elevator Commands///////////////////////////////////////
        NamedCommands.registerCommand("ElevatorLevel3", new ElevatorHeight(elevator, -220));
        NamedCommands.registerCommand("ElevatorLevel2", new ElevatorHeight(elevator, -168));
        NamedCommands.registerCommand("ElevatorLevel1", new ElevatorHeight(elevator, -95));
        NamedCommands.registerCommand("Algae", new ElevatorHeight(elevator, -190));
        NamedCommands.registerCommand("ElevatorPlayerStation", new ElevatorHeight(elevator, -110));

        /////////////////////////////////////Wrist Commands///////////////////////////////////////
        NamedCommands.registerCommand("WristLevel1", new WristHeight(elevator, -15));
        NamedCommands.registerCommand("WristLevel2", new WristHeight(elevator, -18.1));
        NamedCommands.registerCommand("WristLevel3", new WristHeight(elevator, -15.5));
        NamedCommands.registerCommand("WristPlayerStation", new WristHeight(elevator, -8));

        /////////////////////////////////////Shooter Commands///////////////////////////////////////
        NamedCommands.registerCommand("WaitForCoral", new WaitForObjectToBeGrabbed(shooter, 30));
        NamedCommands.registerCommand("Shoot", new Shoot(shooter));
        NamedCommands.registerCommand("autonCommand", new autonCommand1(elevator));
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        


        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    
}
