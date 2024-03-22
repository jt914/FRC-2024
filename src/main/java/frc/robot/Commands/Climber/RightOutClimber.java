package frc.robot.Commands.Climber;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Winch;

public class RightOutClimber extends Command{
    private Winch winch;

    @Override
    public void initialize(){
        winch = Constants.winch;
    }
    @Override
    public void execute(){
        winch.moveRightOut();
    }
    @Override
    public void end(boolean interrupted){
        winch.rightStop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}