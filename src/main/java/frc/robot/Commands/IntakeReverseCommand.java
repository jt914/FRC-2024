package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lights;

public class IntakeReverseCommand extends Command {
    private Arm arm;
    private Intake intake;
    private boolean isFinished = false;
    public Lights lights;
    
    public IntakeReverseCommand(){
        addRequirements(Constants.intake);
        lights = Constants.lights;
    }

    @Override
    public void initialize(){
        intake = Constants.intake;   
    }

    @Override
    public void execute(){
        intake.reverse();
        lights.off();
        Constants.hasNote = false;
    }
    @Override
    public boolean isFinished(){
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {
    // Constants.intakeStatus = false;
        intake.stop();
        

    }
    
}
