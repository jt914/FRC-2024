package frc.robot.Commands;
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

    @Override
    public void initialize(){
        arm = Constants.arm;

    }

    @Override
    public void execute(){
        //right trigger is up
        //left trigger is down
        double rightVal = Constants.alternateController.getRightTriggerAxis();
        double leftVal = Constants.alternateController.getLeftTriggerAxis();

        SmartDashboard.putNumber("armPos", arm.updateAngle());

        if(Constants.alternateController.y().getAsBoolean() == true){
            arm.armEnc.reset();
            System.out.println("resetting");
        }

        if(rightVal > 0.5){
            arm.forward();
        }
        else if (leftVal > 0.5){
            arm.backward();
        }


        // if(leftVal > 0.2 && rightVal > 0.2){
        //     return;
        // }

        // else if(rightVal > 0.2){
        //     arm.setDesired(arm.desiredAngle + (0.1 * rightVal)); //0.1 is max degrees per 20 ms
        // }
        
        // else if (leftVal > 0.2){
        //     arm.setDesired(arm.desiredAngle - (0.1 * leftVal)); //0.1 is max degrees per 20 ms
        // }

        // arm.updateAngle();
        // arm.moveArm();
        
        
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();

    }





    
}


