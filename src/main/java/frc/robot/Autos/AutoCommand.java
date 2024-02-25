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
    private PIDController aimController = new PIDController(.002, 0.000001, 0);
    private PIDController drivePID = new PIDController(1.5, 0.0001, 0);
    private int counter;
    private int step = 1;

    public AutoCommand(){
        swerve = Constants.swerve;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(target != null){
            double xSpeed = drivePID.calculate(target.getX(), 1);
            double ySpeed = drivePID.calculate(target.getY(), -4);

            SmartDashboard.putNumber("xCurrent", target.getX());
            SmartDashboard.putNumber("yCurrent", target.getY());

            SmartDashboard.putNumber("xCalculated", xSpeed);
            SmartDashboard.putNumber("yCalculated", ySpeed);
        }

        target = Constants.camera.getTarget();


        if(step == 1 && target != null){
            counter = 0;
            double xSpeed = drivePID.calculate(target.getX(), 2.5);
            double ySpeed = drivePID.calculate(target.getY(), -0.06);
            if(target.getX() - 2.5 < 0.1 && target.getY() < 0.1){
                step = 2;
                System.out.println("step2");
            }
            swerve.drive(xSpeed, ySpeed, 0);
        }

        if(step == 2){
            if(Constants.m_gyro.getTotalAngleDegrees() < -175){
                step = 3;
            }
            swerve.drive(0,0,aimController.calculate(Constants.m_gyro.getTotalAngleDegrees(), -180));

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

