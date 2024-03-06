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

public class TwoNoteCommand extends Command {
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
    private PIDController turnPID = new PIDController(.002, 0.000001, 0);
    private PIDController drivePID = new PIDController(5, 0.0001, 0);
    private PIDController aimController = new PIDController(.19, 0.000001, 0);
    private int counter = 0;
    private int step = 0;
    private double xSpeed, ySpeed, yaw;
    private Shooter shooter;
    private Arm arm;
    private Intake intake;
    private IntakeCommand intakeCommand;
    private boolean triggered = false;
    private int elapsed = 0;

    public TwoNoteCommand(){
        swerve = Constants.swerve;
        arm = Constants.arm;
        shooter = Constants.shooter;
        intake = Constants.intake;
        
    }
    @Override
    public void initialize(){

  
    }
    //things to do: Figure out if gyro is rotating which direction
    //Find exact locations of the notes
    //Tune PID and Drive PID
    @Override
    public void execute(){
        if(step == 0){
            Constants.arm.setDesired(4);
            counter++;
            if(counter > 50){
                step = 1;
                counter = 0;
            }
        }


        if(step == 1){
            Constants.shooter.setVelocity();
            counter++;
            if(counter > 50){
                step = 2;
                counter = 0;
            }
        }

        if(step == 2){
            intake.run();
            counter++;
            if(counter > 50){
                counter = 0;
                step = 4;
                shooter.stop();
                intake.stop();
            }
        }

    }
    @Override
    public void end(boolean interrupted) {

    }
}

