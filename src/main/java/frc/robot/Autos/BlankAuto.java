package frc.robot.Autos;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class BlankAuto extends Command {
    // private Arm arm;
    // private Intake intake;
    private boolean isFinished = false;
    // private Shooter shooter;
    private Drivetrain swerve;
    private int swerveStep;
    private int shooterStep;
    private int intakeStep;
    private int armStep;
    private Transform3d target;
    // private PIDController turnPID = new PIDController(.002, 0.000001, 0);
    // private PIDController drivePID = new PIDController(5, 0.0001, 0);
    // private PIDController aimController = new PIDController(.19, 0.000001, 0);
    private int counter = 0;
    private int step = 0;
    private double xSpeed, ySpeed, yaw;
    private Shooter shooter;
    private Arm arm;
    private Intake intake;
    private IntakeCommand intakeCommand;
    private boolean triggered = false;
    private int elapsed = 0;
    InterpolatingDoubleTreeMap tm = new InterpolatingDoubleTreeMap();

    public BlankAuto(){

    }
    @Override
    public void initialize(){
    
    }
        
    
    @Override
    public void execute(){
        
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(0,0,0);

    }
}

