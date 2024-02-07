package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class ShooterCommand extends Command{
    /*
     * actions needed:
     * 1st button, should run 'aim' command, then run shooter. Maybe use synchronized commands to speed up execution    
     * 
     * 2nd button, should move shooter position to source intake
     * 
     * 3rd button, if needed, moves shooter to lowered position for moving under the stage
     */


    private boolean finish;
    private double botSpeed = 0.7;
    private double topSpeed = 0.7;
    



    @Override
    public void initialize(){
        SmartDashboard.putNumber("botSpeed", botSpeed);
        SmartDashboard.putNumber("topSpeed", topSpeed);


    }

    
    @Override
    public void execute(){

        // if(Constants.alternateController.rightBumper().getAsBoolean()){
        //     Constants.RPMBot = Constants.RPMBot + 100;
        // }
        // if(Constants.alternateController.leftBumper().getAsBoolean()){
        //     Constants.RPMBot = Constants.RPMBot - 100;
        // }

        // if(Constants.swerveController.leftBumper().getAsBoolean()){
        //     Constants.RPMTop = Constants.RPMTop - 100;
        // }

        // if(Constants.swerveController.rightBumper().getAsBoolean()){
        //     Constants.RPMTop = Constants.RPMTop + 100;
        // }


        // Constants.shooter.setVelocity();

        Constants.shooter.setSpeed(-0.7, -0.85);



        //idea is that the arm and the shooter motors can continuously adjust incrementally 
        //until they reach within a certain threshold
        
        // double[] optimal = Constants.camera.getDesiredShoot();
        // double desiredAngle = Constants.camera.getDriveOffset();

        //margin of errors, should be adjusted
        // SmartDashboard.putNumber("arm", optimal[0] - Constants.arm.updateAngle());
        // // SmartDashboard.putNumber("gyro", Constants.m_gyro.getTotalAngleDegrees()%360 - desiredAngle);

        // if(finish){
        //     Constants.intake.run(); //to shoot the note out
        //     Constants.shooter.setSpeed(optimal[1], optimal[2]);

        // }
        // else{
        //     // if(Math.abs(optimal[0] - Constants.arm.updateAngle()) < 1 && Math.abs(Constants.m_gyro.getTotalAngleDegrees()%360 - desiredAngle) < 5){
        //     finish = true;

        // else{
        //     Constants.arm.setAngle(optimal[0]);
        //     Constants.shooter.setSpeed(optimal[1], optimal[2]);
        // }

        



        // Constants.m_swerve.drive(0,0, desiredAngle);
        
    }

    
    @Override
    public void end(boolean interrupted){
        Constants.shooter.stop();
        
    }


    @Override
    public boolean isFinished() {
        return false;
    }




    
}
