package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class ResetModulesCommand extends Command {
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
