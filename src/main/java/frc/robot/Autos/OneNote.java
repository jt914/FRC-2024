package frc.robot.Autos;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ResetModulesCommand;
import frc.robot.Commands.ToggleAutoAimCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class OneNote extends Command {
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
    private int step = -1;
    private double xSpeed, ySpeed, yaw;
    private Shooter shooter;
    private Arm arm;
    private Intake intake;
    private IntakeCommand intakeCommand;
    private boolean triggered = false;
    private int elapsed = 0;
    public Trigger trig;
    public BooleanSupplier autoAim;
    public boolean aim = false;
    public Camera cam;
    InterpolatingDoubleTreeMap tm = new InterpolatingDoubleTreeMap();


    public OneNote(){
        swerve = Constants.swerve;
        arm = Constants.arm;
        shooter = Constants.shooter;
        intake = Constants.intake;
        autoAim = () -> aim;
        trig = new Trigger(autoAim);
        trig.onTrue(new ToggleAutoAimCommand());
            
    }
    @Override
    public void initialize(){
        tm.put(3.74, 33.0);
        tm.put(2.4, 26.0);
        tm.put(1.3, 15.5);

    }
    @Override
    public void execute(){

        if(step == -1){
            counter++;
            swerve.drive(-0.1, swerve.tunedDriveY(-1.8), 0);
            if(counter > 50){
                step = 0;
                counter = 0;
                swerve.drive(0,0,0);
            }
        }


        if(step == 0){
            shooter.setVelocity();
            Constants.arm.setDesired(6);
            counter++;
            double[] desired = Constants.camera.getDesiredShoot(0);
            if(desired != null){
                swerve.drive(0,0, -1 * aimController.calculate(desired[0], -1));
            }
            if(counter > 100){
                step = 1;
                counter = 0;
            }
        }

        if(step == 1){
            intake.runFast();
            counter++;
            if(counter > 100){
                step = 2;
                counter = 0;
                intake.stop();
                shooter.stop();
                arm.setDesired(5);
            }
        }


        if(step == 2){
            counter++;
            swerve.drive(swerve.tunedDriveX(-6), 0, 0);
            
            if(counter > 50){
                step = 3;
                counter = 0;
                swerve.drive(0,0,0);
            }
        }


   

    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(0,0,0);

    }

}
