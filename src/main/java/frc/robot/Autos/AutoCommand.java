package frc.robot.Autos;
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
        swerveStep = 0;
        shooterStep = 1;
        intakeStep = 0;
        armStep = 0;
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Current Swerve Step: ", swerveStep);
        SmartDashboard.putNumber("XDist", Constants.swerve.m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("YDist", Constants.swerve.m_odometry.getPoseMeters().getY());

        if(swerveStep == 0)
        {
            if(Constants.swerve.m_odometry.getPoseMeters().getX() < 1) {
                Constants.swerve.drive(.7, 0, 0);
            }
            else {
                swerveStep++;
            } 
        }
        if(swerveStep == 1)
        {
            if(Constants.swerve.m_odometry.getPoseMeters().getX() > 0 && shooterStep == 1) {
                Constants.swerve.drive(-.7,0,0);
            }
        }
        if(shooterStep == 0)
        {
            // if(shooter.getSpeedTop()<3000) {
            //     shooter.setSpeed(-0.6, -0.9);
            // }
            // else {
            //     shooterStep++;
            // }
        }
    
       
    }

    @Override
    public void end(boolean interrupted) {

    }






}

