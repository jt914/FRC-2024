package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class IntakeCommand extends Command {

    private boolean isFinished;



    @Override
    public void initialize(){
        if(Constants.intake.running){
            Constants.intake.stop();
            isFinished = true;
        } else{
            Constants.intake.run();
            isFinished = true;
        }
        

    }

    
    @Override
    public void execute(){
    }

    
    @Override
    public void end(boolean interrupted){

        
        
    }


    @Override
    public boolean isFinished() {
        return isFinished;
    }




    
}


