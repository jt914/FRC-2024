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
    InterpolatingDoubleTreeMap tm = new InterpolatingDoubleTreeMap();

    public TwoNoteCommand(){
        swerve = Constants.swerve;

        shooter = Constants.shooter;
        intake = Constants.intake;
        
    }
    @Override
    public void initialize(){
        tm.put(1.90, 12.69);
        tm.put(2.73, 18.238);
        tm.put(3.3, 19.73);
        tm.put(4.7, 23.48);
  
    }
    //things to do: Figure out if gyro is rotating which direction
    //Find exact locations of the notes
    //Tune PID and Drive PID
    @Override
    public void execute(){
        if(step == 0){
            Constants.arm.setDesired(10);
            counter++;
            if(counter > 50){
                step = 1;
                counter = 0;

            }
        }

        if(step == 1){
            counter++;
            swerve.drive(2,0,-3.5);
            if(counter > 15){
                swerve.drive(0,0,0);
                step = 2;
                counter = 0;
            }
        }

        if(step == 2){
            double [] desired = Constants.camera.getDesiredShoot(0);
            Constants.shooter.setVelocity();
            counter++;
            if(desired != null && desired[0] != 0){
                yaw = aimController.calculate(-1 + desired[0], 0);
                Constants.arm.setDesired(15);
                SmartDashboard.putNumber("arm positionwasdwasd", tm.get(desired[1]));
                swerve.drive(0,0,yaw);

            }
            if(counter > 50){
                step = 3;
                counter = 0;

            }
        }

        else if(step == 3){
            intake.run();

            counter++;
            if(counter > 50){
                counter = 0;
                step = 4;
                shooter.stop();
                intake.stop();
                Constants.arm.setDesired(6);
            }
        }

        else if(step == 4){
            swerve.drive(2,0, 0.1);
            intake.run();
            if(intake.hasNote()){
                step = 5;
                swerve.drive(0,0,0);
                intake.stop();
            }
        }

        else if(step == 5){
            double [] desired = Constants.camera.getDesiredShoot(0);
            Constants.shooter.setVelocity();
            counter++;
            if(desired != null && desired[0] != 0){
                yaw = aimController.calculate(-1 + desired[0], 0);
                // Constants.arm.setDesired(tm.get(desired[1]));
                SmartDashboard.putNumber("arm positionwasdwasd", tm.get(desired[1]));
                swerve.drive(0,0,yaw);
            }
            if(counter > 50){
                step = 6;
                counter = 0;
            }
        }
        
        else if(step == 6){
            intake.run();

            counter++;
            if(counter > 50){
                counter = 0;
                step = 7;
                shooter.stop();
                intake.stop();

            }
        }
    }

  
        
        // if(target == null){
        //     counter++;
        // }
        // if(counter > 10){
        //     swerve.drive(0,0,0);
        // }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(0,0,0);

    }
}

