package frc.robot.Autos;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class AutoCommand extends Command {
    // private Arm arm;
    // private Intake intake;
    private boolean isFinished = false;
    // private Shooter shooter;
    private Drivetrain swerve;

    private Transform3d target;
    private PIDController turnPID = new PIDController(.002, 0.000001, 0);
    private PIDController drivePID = new PIDController(5, 0.0001, 0);
    private int counter, shooterCounter;
    private int step = -1;
    private double xSpeed, ySpeed, yaw;
    private Shooter shooter;
    private Intake intake;
    private boolean triggered = false;
    private int elapsed = 0;

    public AutoCommand(){
        swerve = Constants.swerve;

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
        target = Constants.camera.getTarget();
        if(step == -1){
            swerve.drive(3,0, 0);
            counter++;
            if(counter > 20){
                counter = 0;
                step = 0;
                swerve.drive(0,0,0);
            }
        }

        if(target != null){
            counter = 0;
            SmartDashboard.putNumber("xCurrent", target.getX());
            SmartDashboard.putNumber("yCurrent", target.getY());
            SmartDashboard.putNumber("Step", step);
            //shoot
            if(step == 0){
                shooter.setVelocity();
                shooterCounter++;
                if(shooterCounter > 50){
                    shooterCounter = 0;
                    step = 1;

                }
            }

            if(step == 1){
                intake.run();

                shooterCounter++;
                if(shooterCounter > 50){
                    shooterCounter = 0;
                    step = 2;
                    shooter.stop();

                }
            }
            //drive and intake 1st note
            if(step == 2){
                xSpeed = drivePID.calculate(target.getX(), 2.65);
                ySpeed = drivePID.calculate(target.getY(), 0.45);
                // yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);

                SmartDashboard.putNumber("xSpeedwasd", xSpeed);
                SmartDashboard.putNumber("ySpeedwasd", ySpeed);
                System.out.println("doing number2");

                intake.run();

                if(Constants.intake.intakeSensor.getVoltage()<.5) {
                    triggered = true;
                }
                if(triggered == true) {
                    elapsed++;
                }
                if(elapsed > 3) {
                    isFinished = true;
                    elapsed = 0;
                    intake.stop();
                    step = 3;
                    swerve.drive(0,0,0);
                }
            }

            if(step == 3){
                shooter.setVelocity();

                shooterCounter++;
                if(shooterCounter > 50){
                    shooterCounter = 0;
                    step = 4;

                }
            }

            if(step == 4){
                intake.run();

                shooterCounter++;
                if(shooterCounter > 50){
                    shooterCounter = 0;
                    step = 5;
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
    }
    @Override
    public void end(boolean interrupted) {
        swerve.drive(0,0,0);

    }
}

