package frc.robot.Commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

public class ArmCommand extends Command {
    private Arm arm;
    private Intake intake;
    private boolean isFinished = false;
    private boolean forward;

    public ArmCommand(){
    
    }

    @Override
    public void initialize(){
        arm = Constants.arm;
        Constants.arm.desiredAngle = Constants.arm.armEnc.getDistance();

    }
    @Override
    public void execute(){

        Constants.arm.desiredAngle = MathUtil.clamp(Constants.arm.desiredAngle, 0,120);
        
        if(Constants.alternateController.leftTrigger().getAsBoolean()){
            Constants.arm.setDesired(Constants.arm.getAngle() - 0.01);
        }
        else if(Constants.alternateController.rightTrigger().getAsBoolean()){
            Constants.arm.setDesired(Constants.arm.getAngle() + 0.01);
        }

    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();

    }

}
