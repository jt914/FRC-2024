package frc.robot.Autos;
import edu.wpi.first.math.controller.PIDController;
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


    public AutoCommand(){
        swerve = Constants.swerve;
    }


    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        double x = swerve.poseEstimator.getEstimatedPosition().getX();
        double y = swerve.poseEstimator.getEstimatedPosition().getY();
        SmartDashboard.putNumber("x" , x);
        SmartDashboard.putNumber("y" , y);

        swerve.drive(-1 * swerve.drivePIDController.calculate(x, -2), -swerve.drivePIDController.calculate(y, 0), swerve.turnPIDController.calculate(0,0));
        // swerve.drive(swerve.driveSimpleMotorFeedforward.calculate(1, 0), swerve.drivePIDController.calculate(y, 0), swerve.turnPIDController.calculate(0,0));

    
       
    }

    @Override
    public void end(boolean interrupted) {

    }






}

