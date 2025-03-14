package frc.robot.subsystems.Photonvision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Swerve;

public class VisionSim {

    Swerve swerve;

    // A vision system sim labelled as "main" in NetworkTables
    VisionSystemSim visionSim = new VisionSystemSim("main");
    TargetModel targetModel = TargetModel.kAprilTag36h11;

    /** The PhotonCamera used in the real robot code*/
    PhotonCamera cameraBackLeft;
    /**The simulation of this camera. Its values used in real robot code will be updated.*/
    PhotonCameraSim cameraBackLeftSim;

    public VisionSim(Swerve swerve, PhotonCamera backLeftCamera){
        this.swerve = swerve;
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        visionSim.addAprilTags(tagLayout);


        //Set up the back left camera
        this.cameraBackLeft = backLeftCamera;
        cameraBackLeftSim = new PhotonCameraSim(cameraBackLeft, propertiesAprilTagCamera());
        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        // Translation3d robotToBackLeftTrl = new Translation3d(-14, -13.5, 13.5);
        Translation3d robotToBackLeftTrl = new Translation3d(0, 0, 13.5);
        // and pitched 15 degrees up.
        // Rotation3d robotToBackLeftRot = new Rotation3d(0, 0, Degrees.of(180+40).in(Radians));
        Rotation3d robotToBackLeftRot = new Rotation3d(0, 0, Degrees.of(0).in(Radians));
        Transform3d robotToBackLeft = new Transform3d(robotToBackLeftTrl, robotToBackLeftRot);
        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraBackLeftSim, robotToBackLeft);

        cameraBackLeftSim.enableRawStream(false);
        //Enable the really fancy debug stream; computationally expensive!!
        cameraBackLeftSim.enableDrawWireframe(true);
        cameraBackLeftSim.setWireframeResolution(1);

    }

    public void periodic(){
        // sim.getDebugField().setRobotPose(swerve.getPose());
        visionSim.update(swerve.getPose());
    }

    private SimCameraProperties propertiesAprilTagCamera(){ 
        // The simulated camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // // Approximate detection noise with average and standard deviation error in pixels.
        // cameraProp.setCalibError(0.25, 0.08);
        // // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        // cameraProp.setFPS(20);
        // // The average and standard deviation in milliseconds of image data latency.
        // cameraProp.setAvgLatencyMs(35);
        // cameraProp.setLatencyStdDevMs(5);
        return cameraProp;
    }

}


