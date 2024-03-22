package frc.robot.Subsystems;

import java.io.Console;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.*;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private PhotonCamera cam;
    public PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout fieldLayout;
    Transform3d robotToCam = new Transform3d(new Translation3d(0.13, 0.33, 0.35), new Rotation3d(0,35,Math.PI)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    public Camera(){
        cam = new PhotonCamera("Arducam_OV9782_USB_Camera");
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    }

    public PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
        
    }

    public Optional<EstimatedRobotPose> getPose() {
        return photonPoseEstimator.update();
    }

    public void end(){
        cam.close();
    }

    public Transform3d getTarget(){
        PhotonTrackedTarget shooterTarget = null;
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            if(target.getFiducialId() == 7 || target.getFiducialId() == 1){
                shooterTarget = target;
            }
        }
        if(shooterTarget != null){

            return shooterTarget.getBestCameraToTarget();
        }
        return null;
    }
    //Test method, returns true if sees target
    public boolean ifTarget(){
        PhotonTrackedTarget shooterTarget = null;
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            if(target.getFiducialId() == 7 || target.getFiducialId() == 1){
                shooterTarget = target;
            }
        }
        if(shooterTarget != null){

            return true;
        }
        return false;
    }

    //if null the command to get desired shoot should just do nothing
    public double[] getDesiredShoot(double ySpeed){
        double[] retVal = new double[3];

        /*
        1. use Camera to get pose from hood
        2. calculate optimal arm angle and shooter speed for camera
        might need to do testing
        */
        PhotonTrackedTarget shooterTarget = null;
        // PhotonUtils.calculateDistanceToTargetMeters(0, );
        boolean seesShooter = false;
        for(PhotonTrackedTarget target: getLatestResult().getTargets()){
            if(target.getFiducialId() == 7 || target.getFiducialId() == 4){
                shooterTarget = target;
                seesShooter = true;
            }
        }

        if(seesShooter){
            //8
                retVal[0] = shooterTarget.getYaw() + 3 * ySpeed;

            retVal[1] =
            PhotonUtils.calculateDistanceToTargetMeters(
                    0.286,
                    1.4478,
                    Units.degreesToRadians(15),
                    Units.degreesToRadians(shooterTarget.getPitch()));

                


            retVal[2] = shooterTarget.getBestCameraToTarget().getX();
            System.out.println(retVal[2]);

            return retVal;
        }

        return null;
    }

    public double getDriveOffset(){
        /*
        1. use Camera to get pose from hood
        2. calculate rotation needed to make it
        might need to do testing
        
        */
        return 90;
    }
}