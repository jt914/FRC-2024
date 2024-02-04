package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

public class ArmCommand extends Command {
    private Arm arm;
    private Intake intake;
    private boolean isFinished = false;
    private boolean forward;

    public ArmCommand(boolean forward){
        this.forward = forward;

    }


    @Override
    public void initialize(){
        arm = Constants.arm;

        
    }

    @Override
    public void execute(){
        if(forward){
            arm.forward();
        }
        else{
            arm.backward();
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();

    }





    
}


