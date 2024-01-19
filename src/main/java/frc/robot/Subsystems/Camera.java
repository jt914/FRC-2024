package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Camera {
    private PhotonCamera cam;
    public static class LimeLight {
        private NetworkTable table;
        
        public LimeLight() {
            table = NetworkTableInstance.getDefault().getTable("limelight-cam");
        }

        public NetworkTable getTable() {
            return table;
        }

        public double getTid() {
            System.out.println("getTid");
            return table.getEntry("tid").getDouble(0);
        }

        public double[] getTids() {
            return table.getEntry("tid").getDoubleArray(new double[0]);
        }
    }


    public Camera(){
        cam = new PhotonCamera("photonvision");
        
    }

    public PhotonPipelineResult getLatestResult(){
        return cam.getLatestResult();
        
    }

    public void end(){
        cam.close();
    }

}
