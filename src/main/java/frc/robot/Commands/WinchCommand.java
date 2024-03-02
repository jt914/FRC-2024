package frc.robot.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Winch;

public class WinchCommand extends Command {
    private Winch winch;

    public WinchCommand(){
        addRequirements(Constants.winch);
        winch = Constants.winch;

    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){

        if(Constants.swerveController.leftTrigger().getAsBoolean() || Constants.swerveController.rightTrigger().getAsBoolean() || Constants.alternateController.y().getAsBoolean()){
            if(Constants.swerveController.leftTrigger().getAsBoolean()){
                winch.moveLeftIn();
            }
            if(Constants.swerveController.rightTrigger().getAsBoolean()){
                winch.moveRightIn();
            }
            if(Constants.alternateController.y().getAsBoolean()){
                winch.moveOut();
            }
        }

        else{
            winch.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
