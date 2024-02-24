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
    private PIDController aimController = new PIDController(.23, 0.000001, 0);
    private PIDController drivePID = new PIDController(2, 0, 0);


    public AutoCommand(){
        swerve = Constants.swerve;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        target = Constants.camera.getTarget();
        double xSpeed = drivePID.calculate(target.getX(), 1.44);
        double ySpeed = drivePID.calculate(target.getY(), 1.77);
        SmartDashboard.putNumber("xCalculated", xSpeed);
        SmartDashboard.putNumber("yCalculated", ySpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }
}

