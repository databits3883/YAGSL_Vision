package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionSubsystem  extends SubsystemBase {
   private PhotonCamera camera;
   private AprilTagFieldLayout aprilTagFieldLayout;
   private PhotonPoseEstimator photonPoseEstimator;
   private PhotonPipelineResult pipelineResult;

   public VisionSubsystem(double cameraFrontToBackInMeters, double cameraSideToSideInMeters, double cameraHeightInMeters, String cameraName) {
      System.out.println("Vision: About to connect to camera");
      this.camera = new PhotonCamera(cameraName);
      System.out.println("Vision: got Camera: " + this.camera.getName());
      pipelineResult = new PhotonPipelineResult();
      
      try {
         this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
         System.out.println("Unable to load AprilTagFieldLayout: " + e.toString());
         e.printStackTrace(System.err);
         this.aprilTagFieldLayout = null;
      }
      System.out.println("Vision: got AprilTagFieldLayout: " + this.aprilTagFieldLayout);
      
      //Setup Pipeline - 0 == AprilTag pipeline
      camera.setPipelineIndex(0);

      Transform3d robotToCam = new Transform3d(new Translation3d(cameraFrontToBackInMeters, cameraSideToSideInMeters, cameraHeightInMeters), new Rotation3d(0.0D, 0.0D, 0.0D));
      this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, robotToCam); 
      System.out.println("Vision: Setup Pose Estimator: " + this.photonPoseEstimator);
   }

   public Pose2d debugClosestTarget() {
      Pose2d estimatedRobotPose = null;
      PhotonTrackedTarget target = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();
      System.out.println("vision: Got target: " + hasTargets);
      if (hasTargets) {
         target = result.getBestTarget();
         double yaw = target.getYaw();
         double pitch = target.getPitch();
         double area = target.getArea();
         double skew = target.getSkew();
         List<TargetCorner> corners = target.getMinAreaRectCorners();
         //System.out.println("Vision: got target data: (yaw/pitch/area/skew) (" + yaw + "/" + pitch + "/" + area + "/" + skew + ")");
         int targetID = target.getFiducialId();
         System.out.println("vision: got aprilTag id: " + targetID);
         double poseAmbiguity = target.getPoseAmbiguity();
         System.out.println("vision: got poseAmbiguity: " + poseAmbiguity);
         Transform3d bestCameraToTarget = target.getBestCameraToTarget();
         System.out.println("bestCameraToTarget X/Y/Rotate: " + bestCameraToTarget.getX() + " / " + bestCameraToTarget.getY() + "/" + bestCameraToTarget.getRotation());
         Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
         //System.out.println("alternateCameraToTarget X/Y/Rotate: " + alternateCameraToTarget.getX() + " / " + alternateCameraToTarget.getY() + "/" + alternateCameraToTarget.getRotation());
         Optional<Pose3d> aprilPoseOptional = aprilTagFieldLayout.getTagPose(targetID);
         if (aprilPoseOptional.isPresent()) {
            estimatedRobotPose = aprilPoseOptional.get().toPose2d();
System.out.println("AprilTag Position X/Y/Rotate: " + estimatedRobotPose.getX() + " / " + estimatedRobotPose.getY() + "/" + estimatedRobotPose.getRotation());  
               // -90
            estimatedRobotPose = new Pose2d(estimatedRobotPose.getX() - bestCameraToTarget.getX(), 
                                          estimatedRobotPose.getY() + bestCameraToTarget.getY(),
                                          Rotation2d.fromDegrees(estimatedRobotPose.getRotation().getDegrees() + 180 + bestCameraToTarget.getRotation().toRotation2d().getDegrees()));
System.out.println("Estimated Robot Position X/Y/Rotate: " + estimatedRobotPose.getX() + " / " + estimatedRobotPose.getY() + "/" + estimatedRobotPose.getRotation()); 
            Optional<EstimatedRobotPose> globalVision = getEstimatedGlobalPose();
            if (globalVision != null && globalVision.isPresent()) {
               Pose2d globalVisionPose = globalVision.get().estimatedPose.toPose2d();
               System.out.println("Estimated Robot Position X/Y/Rotate: " + globalVisionPose.getX() + " / " + globalVisionPose.getY() + "/" + globalVisionPose.getRotation());  
            }
         }
      } else {
         System.out.println("Vision: number of targets: 0");
      }
      return estimatedRobotPose;

   }

   public PhotonTrackedTarget getTarget() {
      PhotonTrackedTarget target = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();

      if(hasTargets) {
         target = result.getBestTarget();
      }
      return target;
   }

   public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      return photonPoseEstimator.update(pipelineResult);
   }

   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousPose) {

      photonPoseEstimator.setReferencePose(previousPose);
      return photonPoseEstimator.update();
   }

   public Optional<Pose3d> getAprilTagPose(int id) {
      return aprilTagFieldLayout.getTagPose(id);
   }

   @Override
   public void periodic() {
       pipelineResult = camera.getLatestResult();
   }   
}