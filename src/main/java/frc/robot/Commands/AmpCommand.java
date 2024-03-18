package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Shooter;

public class AmpCommand extends Command {
    private Arm arm;
    private Shooter shooter;
    
    public AmpCommand(){
        arm = Constants.arm;
        shooter = Constants.shooter;
        addRequirements(arm);
        addRequirements(shooter);
        addRequirements(Constants.intake);
    }

    @Override
    public void initialize(){
   
    }
    @Override
    public void execute(){
        arm.setDesired(105);

        if(arm.getMeasurement() > 45) {
            shooter.setLowVelocity();
        }
    }

    @Override
    public void end(boolean interrupted) {
        Constants.shooter.stop();

    }
}
