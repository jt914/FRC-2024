package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterCommand extends Command{

    @Override
    public void initialize(){
 
    }

    @Override
    public void execute(){
        Constants.shooter.setVelocity();

    }

    
    @Override
    public void end(boolean interrupted){
        Constants.shooter.stop();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
  
}