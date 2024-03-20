package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Winch;

public class WinchCommand extends Command{
    Winch winch;
    @Override
    public void initialize(){
        winch = Constants.winch;
    }

    @Override
    public void execute(){
        if(Constants.alternateController.leftTrigger().getAsBoolean()) {
            winch.moveLeftIn();
        }
        if(Constants.alternateController.rightTrigger().getAsBoolean()) {
            winch.moveRightIn();
        }
        if(Constants.alternateController.leftBumper().getAsBoolean()) {
            winch.moveLeftOut();
        }
        if(Constants.alternateController.rightBumper().getAsBoolean()) {
            winch.moveRightOut();
        }
        if(Constants.alternateController.b().getAsBoolean()) {
            winch.stop();
        }
    }
    @Override
    public void end(boolean interrupted){
        winch.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}