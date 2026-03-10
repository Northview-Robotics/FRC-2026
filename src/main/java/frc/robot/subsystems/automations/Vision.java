package frc.robot.subsystems.automations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
//General imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Drive;

import java.util.List;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
//Vision imports
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.SimCameraProperties;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision extends SubsystemBase{
    private static Vision vision = null;

    //Vision objects
    private VisionSystemSim visionSim;
    private final Transform3d cameraPos;
    private AprilTagFieldLayout field;   
    private TargetModel targetModel;

    //Vision parameters
    private double t_Width = 0.5;
    private double t_Height = 0.25;

    //Advantage Scope
    private StructArrayPublisher<Pose3d> as_aprilTags;
    private StructPublisher<Pose3d> as_cameraPose; //Camera pose relative to robot on the field
    private StructPublisher<Pose2d> as_estimatedCameraPose;
    private Pose3d[] aprilTagList;

    //Pose estimation
    private PoseStrategy strat;
    private final PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> currentStdDevs;


    //Camera parameters
    private SimCameraProperties cameraProp;
    private int c_Width = 640;
    private int c_Height = 480;
    private double c_Error = 0.2;
    private double fov = 70;
    private double fps = 20;
    private double latencyAve = 50; //Average latency
    private double latencySTD = 15; //Standard deviation of latency
    private Rotation2d c_FOV = Rotation2d.fromDegrees(fov);

    //Camera objects
    private PhotonCameraSim cameraSim;
    private PhotonCamera camera;
    
    //Target info
    int tagId;

    //Constructor for all vision initializations
    private Vision(){
        //Field Setup
        field = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        //Camera setup
        cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(c_Width, c_Height, c_FOV);
        cameraProp.setCalibError(c_Error, c_Error);
        cameraProp.setFPS(fps);
        cameraProp.setAvgLatencyMs(latencyAve);
        cameraProp.setLatencyStdDevMs(latencySTD);

        camera = new PhotonCamera("cam1");
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraPos = new Transform3d(
            new Translation3d(VisionConstants.camPosX, VisionConstants.camPosY, VisionConstants.camPosZ), //Position of camera on the robot
            new Rotation3d(0, VisionConstants.camRotPitch, 0) //Rotate the camera POV
        );
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        //Simulation
        visionSim = new VisionSystemSim("main");
        visionSim.addCamera(cameraSim, cameraPos);
        visionSim.addAprilTags(field);
        targetModel = new TargetModel(t_Width, t_Height);
        
        //Pose estimation tools
        strat = PoseStrategy.AVERAGE_BEST_TARGETS;
        poseEstimator = new PhotonPoseEstimator(field, strat, cameraPos);

        //Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault().getStructArrayTopic("aprilTags", Pose3d.struct).publish();
        as_cameraPose = NetworkTableInstance.getDefault().getStructTopic("cameraPose", Pose3d.struct).publish();
        as_estimatedCameraPose = NetworkTableInstance.getDefault().getStructTopic("estimatedPose", Pose2d.struct).publish();
    }

    //Main vision code
    public void updateVision() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for(var result : camera.getAllUnreadResults()){
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if(visionEst.isEmpty()){
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
                        Drive.getInstance().swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if(estimatedPose.isEmpty()){
            //Set to default std (standard deviation) value
            currentStdDevs = VisionConstants.kSingleTagStdDevs;
        } else {
            //Initialize calculation by assuming default std, tags and distance (standard deviation)
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDistance = 0;

            //Go through all tags and count them and their distances relative to the robot
            for(var target : targets){
                var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDistance += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if(numTags == 0){
                // No visable tags. Default to singletag std devs
                currentStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible in this case
                
                //Distribute average distance between tags
                avgDistance /= numTags;
                // Decrease std if multiple targets are visible, because it's more accurate than single target
                if(numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                if(numTags == 1 && avgDistance > 4)
                    //Std is set to the max if the average distance is more than 4m because the vision value will be the least accurate
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDistance * avgDistance / 30)); //Increase std linearly based on average distance less than 4m
                currentStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }

    //Simulation Code
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    public void updateSimVision(Pose2d botPose){
        visionSim.update(botPose);
        Optional<Pose3d> camPose = visionSim.getCameraPose(cameraSim);
        camPose.ifPresent(as_cameraPose::set);
        as_aprilTags.set(field.getTags().stream().map(tag -> tag.pose).toArray(Pose3d[]::new));
    }

    public void resetVision(Pose2d resetPose){
        visionSim.resetRobotPose(resetPose);
        visionSim.clearCameras(); //Problematic line
        visionSim.clearVisionTargets(); //Problematic line
    }

    //Helper methods

    public Optional<EstimatedRobotPose> getVisionPose() {
        var result = camera.getLatestResult();
        Optional<EstimatedRobotPose> estPose = poseEstimator.estimateCoprocMultiTagPose(result);

        if (estPose.isEmpty()) {
            estPose = poseEstimator.estimateLowestAmbiguityPose(result);
            camera.getAllUnreadResults();
        }

        return estPose;
    }

    public int getTargetAprilTag(){
        var result = camera.getLatestResult();
        double areaMax = 0;
        for(PhotonTrackedTarget target : result.getTargets()){
            if(target.getArea() > areaMax){
                tagId = target.fiducialId;
                areaMax = target.getArea();
            }
        }
        
        return tagId;
    }

    public static Vision getInstance(){
        if (vision == null){
            vision = new Vision();
        }
        return vision;
    }
}