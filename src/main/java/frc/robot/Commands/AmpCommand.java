package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class AmpCommand extends Command {
    private Arm arm;
    private Intake intake;
    private Shooter shooter;
    private boolean isFinished = false;
    private boolean forward;

    public AmpCommand(){

    }


    @Override
    public void initialize(){
        arm = Constants.arm;
        intake = Constants.intake;
        shooter = Constants.shooter;
    }

    @Override
    public void execute(){
        arm.setDesired(100);
        arm.moveArm();
        shooter.setSpeed(-.4, -0.55);
        // if(shooter.getSpeedBottom() > 2000 && arm.armEnc.getDistance() > 95) {
        //     intake.run(.5);
        // }
        
    }

    @Override
    public void end(boolean interrupted) {

    }






}

