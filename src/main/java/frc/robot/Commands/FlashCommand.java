package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lights;

public class FlashCommand extends Command {

    boolean isFinished;
    int time;
    Lights lights = Constants.lights;
    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        if(time % 40 == 0) {
            lights.off();
        }
        else if (time % 40 == 20) {
            lights.previous();
        }
        time++;
    }
    @Override
    public boolean isFinished(){
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {


    }
    
}
