package frc.robot.Commands;
import java.util.function.IntConsumer;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class ResetModulesCommand extends Command {
    private Arm arm;
    private Shooter shooter;
    private Drivetrain swerve;
    
    public ResetModulesCommand(){
        swerve = Constants.swerve;
    }

    @Override
    public void initialize(){
   
    }
    @Override
    public void execute(){
        swerve.resetAbsoluteModules();
    }
    @Override
    public void end(boolean interrupted) {

    }
}