package frc.robot.Autos;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    private int swerveStep;
    private int shooterStep;
    private int intakeStep;
    private int armStep;
    private Transform3d target;
    private PIDController turnPID = new PIDController(.002, 0.000001, 0);
    private PIDController drivePID = new PIDController(1.5, 0.0001, 0);
    private int counter, shooterCounter;
    private int step = 1;
    private double xSpeed, ySpeed, yaw;
    private Shooter shooter;
    private Arm arm;
    private Intake intake;

    public AutoCommand(){


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

        if(target != null){
            counter = 0;
            SmartDashboard.putNumber("xCurrent", target.getX());
            SmartDashboard.putNumber("yCurrent", target.getY());
            SmartDashboard.putNumber("Step", step);
            
            //shoot
            if(step == 1){
                shooter.setLowVelocity();
                shooterCounter++;
                if(shooterCounter > 10){
                    shooterCounter = 0;
                    step = 2;
                    shooter.stop();
                }

            }
            //drive and intake 1st note
            if(step == 2){
                xSpeed = drivePID.calculate(target.getX(), 2.65);
                ySpeed = drivePID.calculate(target.getY(), 1.03);
                yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);
                intake.run();

                if(Math.abs(target.getX()) - 2.65 < 0.1 && Math.abs(target.getY()) - 1.03 < 0.1){
                    step = 3;
                    intake.stop();
                }

            }

            //drive to center
            if(step == 3){
                if(Math.abs(target.getX()) - 2 < 0.1 && Math.abs(target.getY() - 0) < 0.1){
                    step = 4;
                }
                xSpeed = drivePID.calculate(target.getX(), 2);
                ySpeed = drivePID.calculate(target.getY(), 0);
                yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());

                //yaw = aimController.calculate(-1 + desired[0], 0);
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);
            }

            //shoot
            if(step == 4){
                shooter.setLowVelocity();
                shooterCounter++;
                if(shooterCounter > 10){
                    shooterCounter = 0;
                    step = 5;
                    shooter.stop();
                }
            }

            //drive to intake 2nd note
            if(step == 5){
                xSpeed = drivePID.calculate(target.getX(), 2.7);
                ySpeed = drivePID.calculate(target.getY(), 0);
                yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);
                intake.run();

                if(Math.abs(target.getX()) - 2.7 < 0.1 && Math.abs(target.getY()) - 0 < 0.1){
                    step = 6;
                    intake.stop();
                }

            }

            //shoot
            if(step == 6){
                shooter.setLowVelocity();
                shooterCounter++;
                if(shooterCounter > 10){
                    shooterCounter = 0;
                    step = 7;
                    shooter.stop();
                }
            }

            //drive to next location
            if(step == 7){
                if(Math.abs(target.getX()) - 2 < 0.1 && Math.abs(target.getY()) - 1 < 0.1){
                    step = 8;
                }
                xSpeed = drivePID.calculate(target.getX(), 2);
                ySpeed = drivePID.calculate(target.getY(), -1.03);
                yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);
            }

            //intake note
            if(step == 8){
                xSpeed = drivePID.calculate(target.getX(), 2.65);
                ySpeed = drivePID.calculate(target.getY(), -1.03);
                yaw = -1 * turnPID.calculate(-120, Constants.m_gyro.getTotalAngleDegrees());
                yaw = 0;
                swerve.drive(xSpeed, ySpeed, yaw);
                intake.run();
                if(target.getX() - 2.65 < 0.1 && Math.abs(target.getY()) -1 < 0.1){
                    intake.stop();
                    step = 9;
                }

                
            }

            //shoot
            if(step == 9){
                shooter.setLowVelocity();
                shooterCounter++;
                if(shooterCounter > 10){
                    shooterCounter = 0;
                    step = 10;
                    shooter.stop();
                }
            }
        }

        
        if(target == null){
            counter++;
        }
        if(counter > 10){
            swerve.drive(0,0,0);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}

