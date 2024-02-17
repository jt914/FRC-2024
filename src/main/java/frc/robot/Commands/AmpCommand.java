package frc.robot.Commands;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class AmpCommand extends Command {
    private Arm arm;
    private Shooter shooter;
    private Drivetrain swerve;
    private Camera cam;
    private boolean isFinished = false;
    private int ampStep = 0;
    private double distanceTarget;
    private double distanceWall;
    public AmpCommand(){
        arm = Constants.arm;
        swerve = Constants.swerve;
        shooter = Constants.shooter;
        cam = Constants.camera;
        addRequirements(arm);
        addRequirements(swerve);
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
        // PhotonTrackedTarget shooterTarget = null;
        // distanceTarget = PhotonUtils.calculateDistanceToTargetMeters(.35, 1.36525, 35 * Math.PI / 180, Units.degreesToRadians(shooterTarget.getPitch()));
        // for(PhotonTrackedTarget target: cam.getLatestResult().getTargets()){
        //     if(target.getFiducialId() == 7 || target.getFiducialId() == 4){
        //         shooterTarget = target;
        //     }
        // }
        // distanceWall = Math.sqrt(((distanceTarget*distanceTarget)-1.0307325625));
        // double xDist = Math.cos(shooterTarget.getYaw()) * distanceWall;
        // double yDist = Math.sin(shooterTarget.getYaw()) * distanceWall;
        
        
    }
    @Override
    public void execute(){
        // SmartDashboard.putNumber("distanceTarget", distanceTarget);
        // SmartDashboard.putNumber("distanceWall", distanceWall);
        if(ampStep == 0) {
            arm.setDesired(90);
            arm.moveArm();
            ampStep++;
        }
        else if(ampStep == 1 && (arm.getArmEncoder() > 45)) {
            shooter.setLowVelocity();
        }
        
    }

    @Override
    public void end(boolean interrupted) {

    }






}
