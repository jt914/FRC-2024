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
        SmartDashboard.putBoolean("ArmSwitch", arm.armSwitch.get());
        SmartDashboard.putNumber("Arm Encoder: ", arm.armEnc.getDistance());
        SmartDashboard.putNumber("Raw Arm Encoder", arm.armEnc.get());
        SmartDashboard.putNumber("Absolute Distance", arm.armEnc.getAbsolutePosition());

        // if(Constants.alternateController.getRightTriggerAxis() > 0.2){
        //     arm.forward();
        // }

        // else if(arm.armSwitch.get() == true){
        //     arm.stall();
        //     return;
            
        // }

        // else if (Constants.alternateController.getLeftTriggerAxis() > 0.2){
        //     arm.backward();
        // }
        // else {
        //     arm.stall();
        // }

        Constants.arm.desiredAngle = MathUtil.clamp(Constants.arm.desiredAngle, 0,110);
        

        if(Constants.alternateController.leftBumper().getAsBoolean()){
            Constants.arm.setDesired(Constants.arm.desiredAngle - 0.75);
            System.out.println("Moving Down");
        }
        else if(Constants.alternateController.rightBumper().getAsBoolean()){
            Constants.arm.setDesired(Constants.arm.desiredAngle + 0.75);
            System.out.println("Moving Up");
        }

        Constants.arm.moveArm();

    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();

    }






}

