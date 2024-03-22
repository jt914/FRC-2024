package frc.robot.Commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        boolean leftTrigger = Constants.alternateController.leftTrigger().getAsBoolean();
        boolean rightTrigger = Constants.alternateController.rightTrigger().getAsBoolean();
        boolean leftBumper = Constants.alternateController.leftBumper().getAsBoolean();
        boolean rightBumper = Constants.alternateController.rightBumper().getAsBoolean();
        SmartDashboard.putNumber("leftWinchEncoder", winch.winchLeftEncoder.getPosition());
        SmartDashboard.putNumber("rightWinchEncoder", winch.winchRightEncoder.getPosition());


        if(leftTrigger || leftBumper){
            if(Constants.alternateController.leftTrigger().getAsBoolean()) {
                if(winch.winchRightEncoder.getPosition() < 85) {
                    winch.moveLeftOut();
                }
            }
            if(Constants.alternateController.leftBumper().getAsBoolean()) {
                if(winch.winchLeftEncoder.getPosition() > 0) {
                    winch.moveLeftIn();
                }
            }
        }
        else{
            winch.leftStop();
        }

        
        if(rightTrigger || rightBumper){
            if(Constants.alternateController.rightTrigger().getAsBoolean()) {
                if(winch.winchRightEncoder.getPosition() < 72) {
                    winch.moveRightOut();
                }
            }
            if(Constants.alternateController.rightBumper().getAsBoolean()) {
                if(winch.winchLeftEncoder.getPosition() > 0) {
                    winch.moveRightIn();
                }
            }
        }
        else{
            winch.rightStop();
        }

        
    }
    @Override
    public void end(boolean interrupted){
        winch.leftStop();
        winch.rightStop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}