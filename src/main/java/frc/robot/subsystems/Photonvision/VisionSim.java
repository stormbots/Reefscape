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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class VisionSim implements Sendable{

    Swerve swerve;

    // A vision system sim labelled as "main" in NetworkTables
    VisionSystemSim visionSim = new VisionSystemSim("main");
    TargetModel targetModel = TargetModel.kAprilTag36h11;

    /** The PhotonCamera used in the real robot code*/
    PhotonCamera camera;
    /**The simulation of this camera. Its values used in real robot code will be updated.*/
    PhotonCameraSim cameraSim;

    public VisionSim(Swerve swerve, PhotonCamera camera,Transform3d cameraTransform ){
        this.swerve = swerve;
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        visionSim.addAprilTags(tagLayout);

        //Set up the back left camera
        this.camera = camera;
        cameraSim = new PhotonCameraSim(camera, propertiesAprilTagCamera());
        visionSim.addCamera(cameraSim, cameraTransform);

        cameraSim.enableRawStream(true);
        //Enable the really fancy debug stream; computationally expensive!!
        cameraSim.enableDrawWireframe(true);
        wireframe=0.5;
        cameraSim.setWireframeResolution(wireframe);

        SmartDashboard.putData("vision/sim",this);
    }

    public void periodic(){
        // sim.getDebugField().setRobotPose(swerve.getPose());
        visionSim.update(swerve.getPose());
    }

    private SimCameraProperties propertiesAprilTagCamera(){ 
        // The simulated camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(100));
        cameraProp.setFPS(20);
        // // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        return cameraProp;
    }


    double wireframe=1;
    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.addBooleanProperty("Raw Stream",()->true,cameraSim::enableRawStream);
        // builder.addBooleanProperty("Processed Stream",()->true,cameraSim::enableProcessedStream);
        // builder.addBooleanProperty("Wireframes",()->true,cameraSim::enableDrawWireframe);
        builder.addDoubleProperty("Wireframe Resolution",
            ()->wireframe,
            (v)->{wireframe=v;cameraSim.setWireframeResolution(wireframe);}
        );
    }

}


