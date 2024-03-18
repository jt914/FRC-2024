package frc.robot.Commands;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lights;

public class IntakeShootCommand extends Command {
    private Intake intake;
    private boolean isFinished = false;
    public Lights lights;
    int elapsed = 0;

    public IntakeShootCommand(){
        addRequirements(Constants.intake);
        lights = Constants.lights;

    }

    @Override
    public void initialize(){
        intake = Constants.intake;
        elapsed = 0;
    }

    @Override
    public void execute(){
        elapsed++;
        if(Constants.hasNote) {
            lights.off();
        }
        intake.runFast();
        Constants.hasNote = false;
        if(elapsed > 20) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;

    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();

    }
}
