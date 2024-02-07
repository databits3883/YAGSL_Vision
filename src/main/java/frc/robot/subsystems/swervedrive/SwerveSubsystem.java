// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private long lastLoggedTime = 0;
  private long logEventTime = 1000 * 2; // Log events every two seconds
  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(14.5);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // double angleConversionFactor =
    // 0;//SwerveMath.calculateDegreesPerSteeringRotation(12.8);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    // In this case the wheel diameter is 4 inches, which must be converted to
    // meters to get meters/second.
    // The gear ratio is 6.75 motor revolutions per wheel rotation.
    // The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactor\": {");
    System.out.println("\t\"angle\": " + angleConversionFactor + ",");
    System.out.println("\t\"drive\": " + driveConversionFactor);
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.
      
    //rotate bot 180 if in red alliance


    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red))
    {
      //todo
      //swerveDrive.setGyro(new Rotation3d().);
      //swerveDrive.setGyroOffset(new Rotation2d().fromDegrees(180));
    }


    setupPathPlanner();
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0),
            // Translation PID constants
            new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                swerveDrive.swerveController.config.headingPIDF.i,
                swerveDrive.swerveController.config.headingPIDF.d),
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName       PathPlanner path name.
   * @param setOdomToStart Set the odometry position to the start of the path.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getPathCommand(String pathName, boolean setOdomToStart) {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    if (setOdomToStart) {
      // Get the initial pose with alliance switching
      Pose2d initialPose = convertToRedSide(path.getPreviewStartingHolonomicPose());
      resetOdometry(initialPose);
    }
 
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  }

   public Command getAutonomousCommand(String autoName, boolean setOdomToStart) {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(autoName);

    if (setOdomToStart) {
      // Get the initial pose with alliance switching
      Pose2d initialPose = convertToRedSide(path.getPreviewStartingHolonomicPose());
      resetOdometry(initialPose);
    }
 
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return AutoBuilder.followPath(path);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
    // correction for this kind of control.
    return run(() -> {
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
          translationY.getAsDouble(),
          rotation.getAsDouble() * Math.PI,
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
          Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
          Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {
    // Only run vision when in teleop
    // TODO: Perhaps only run vision position based on button?

    if (RobotState.isEnabled() && RobotState.isTeleop()) {
      // See if we log this event
      StringBuffer logOutput = new StringBuffer(500);
      boolean isLogging = false;
      long currentTimeInMs = System.currentTimeMillis();
      if (lastLoggedTime + logEventTime < currentTimeInMs) {
        lastLoggedTime = currentTimeInMs;
        isLogging = true;
      }

      if (1==0){
      PhotonTrackedTarget target = RobotContainer.robotVision.getTarget();
      if (target != null) {
        int id = target.getFiducialId();
        Optional<Pose3d> hasTargetPose = RobotContainer.robotVision.getAprilTagPose(id);
        if (hasTargetPose.isPresent()) {
          Pose3d targetPose = hasTargetPose.get();
          // System.out.println("targetPose X/Y: " + targetPose.getX() + " / " +
          // targetPose.getY());

          Optional<EstimatedRobotPose> opRobotPose = RobotContainer.robotVision.getEstimatedGlobalPose();

          if (opRobotPose != null) {
            if (isLogging)
              logOutput.append("\r\nGot New Position from April tag id: " + id);
            if (opRobotPose.isPresent()) {
              EstimatedRobotPose estimatedRobotPose = opRobotPose.get();
              if (isLogging)
                logOutput.append("\r\nestimatedRobotPose X/Y: " + estimatedRobotPose.estimatedPose.getX() + " / "
                    + estimatedRobotPose.estimatedPose.getY());
              addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          } else {
            if (isLogging)
              logOutput.append("\r\nApril Tag seen with no POSE NOT PRESENT!!");
          }
        }

        // Log the event only if we have a target
        if (isLogging && !logOutput.isEmpty())
          System.out.println(logOutput.toString());

      }
    }//enmd if 1==0
    }

  }

  /**
   * Drive to a given pose using the PathPlanner code
   * 
   * @param pose
   * @return
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 4.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    // if (m_offset != 0)
    //if (RobotContainer.isRedAlliance())
      //setGyroOffset(180);
    // this.setGyroOffset(m_offset);
    // else
    swerveDrive.zeroGyro();
  }

  public void setGyroOffset(double offset) {

    // Rotation3d newRotation =
    // resetOdometry(new Pose2d(getPose().getTranslation(), new
    // Rotation2d().fromDegrees(offset)));
    // m_offset = offset;
    swerveDrive.setGyroOffset(new Rotation3d(0, 0, offset));
  }

  public void calibrate(double angle) {
    swerveDrive.resetDriveEncoders();

    // m_allCalibrated = true;

    // Rotation2d defaultGyro
    // swerveDrive.setGyroOffset(angle);
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW
   * positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Estimate robot pose through the use of april tags
   */
  public void visionPose() {
    long currentTimeMillis = System.currentTimeMillis();
    Pose2d debugTarget = RobotContainer.robotVision.debugClosestTarget();
    if(debugTarget != null){
    //  swerveDrive.addVisionMeasurement(debugTarget, currentTimeMillis);
    swerveDrive.resetOdometry(debugTarget);
    }
  }



  public void addVisionMeasurement(Pose2d robotPose, double timeStamp) {
    swerveDrive.addVisionMeasurement(robotPose, timeStamp);

  }

  /**
   * Alter the pose from Blue to Red for Path Planning when not auto converted
   * ie, on start of auto
   * 
   * @param pose
   * @return
   */
  public Pose2d convertToRedSide(Pose2d pose) {
    // If the robot is marked as being on the RED alliance, flip X on pose to red
    // side
    Optional<DriverStation.Alliance> m_alliance = DriverStation.getAlliance();
    boolean isRedAlliance = (DriverStation.getAlliance().isPresent() && (m_alliance.get() == Alliance.Red));

    if (isRedAlliance) {
      return new Pose2d((Constants.FIELD_WIDTH - pose.getX()), pose.getY(),
          pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    return pose;
  }


  /**
   * Test method to turn to aim at target
   * @param camera
   * @return
   */
  public Command aimAtTarget(PhotonTrackedTarget target)
  {
    return run(() -> {
      if (target != null)
      {
        drive(getTargetSpeeds(0,
                              0,
                              Rotation2d.fromDegrees(target.getYaw()))); // Not sure if this will work, more math may be required.
      }
    });
  }

}
