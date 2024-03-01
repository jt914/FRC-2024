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

    public ArmCommand(){
        addRequirements(Constants.arm);

    }

    @Override
    public void initialize(){
        arm = Constants.arm;

    }
    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();

    }
}
