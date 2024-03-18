package frc.robot.Commands;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ToggleAutoAimCommand extends Command {
    public ToggleAutoAimCommand(){
    }

    @Override
    public void initialize(){
        Constants.autoAim = !Constants.autoAim;
        if(Constants.autoAim == false){
            Constants.shooter.stop();
        }

    }
    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
