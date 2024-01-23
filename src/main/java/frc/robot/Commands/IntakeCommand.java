package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class IntakeCommand extends Command {




    @Override
    public void initialize(){
        if(Constants.intake.running){
            Constants.intake.stop();
        } else{
            Constants.intake.run();
        }
        

    }





    @Override
    public boolean isFinished() {
        return true;
    }




    
}


