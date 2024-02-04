package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ClimbCommand extends Command{
    boolean isFinished;
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
       Constants.arm.climbUp();
    }
    @Override
    public boolean isFinished(){

        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Constants.arm.stop();
    }
}
