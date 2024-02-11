package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class AutoCommand extends Command{

    @Override
    public void initialize(){

    }

    
    @Override
    public void execute(){

        System.out.println(Constants.camera.getDesiredShoot());
        
        if(Constants.camera.getDesiredShoot() < 10){
            Constants.intake.run();
        }
        
        // System.out.println("working");


          
      
    
    }

    
    @Override
    public void end(boolean interrupted){
        
    }

}
