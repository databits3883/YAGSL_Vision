// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double ROBOT_MAX_SPEED = Units.feetToMeters(14.5);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double FIELD_WIDTH = Units.inchesToMeters(653.2);  // (76.1 + 250.5 ) * 2 = 653.2 inches

  public static final String ROBOT_FROGGY_CONFIG_LOCATION = "swerve/neo";

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class OIConstants 
  {
    public static final int kDriverControllerPort = 0;  
    public static final double kDriveStickPower = 2;
  }

  public static final class DriveConstants 
  {
    public static final double kMaxSpeedMetersPerSecond = 4.267;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  }

  public static final class VisionConstants
  {
    public static final boolean hasCamera = true;
    //Camera name in network tables
    public static final String cameraName = "Camera_Module_v1";
    //Position of the camera from center of the robot in meters
    public static final double cameraZ = Units.inchesToMeters(14.25);
    public static final double cameraX = Units.inchesToMeters(1);
    public static final double cameraY = Units.inchesToMeters(10);
    //Pipeline name in network tables
    public static final String pipelineName = "apriltag";

    //How accurate the bot needs the estimated position to be to update field pos (lower = better)
    public static final double acceptibleAmbiguity = 0.25;

    //If false, will not automatically update our pose
    public static final boolean autoUpdatePose = true;
  }

  public static final class PoseConstants
  {
    public static final Pose2d chainID15 = new Pose2d(4.45, 4.94, Rotation2d.fromDegrees(-60));
    public static final Pose2d chainID14 = new Pose2d(5.86, 4.11, Rotation2d.fromDegrees(180));
    public static final Pose2d chainID16 = new Pose2d(4.45, 3.27, Rotation2d.fromDegrees(60));

    public static final Pose2d blueOnePose = new Pose2d(2,7,Rotation2d.fromDegrees(0));
    public static final Pose2d redOnePose = new Pose2d(Constants.FIELD_WIDTH - 2, 7, Rotation2d.fromDegrees(180));
    public static final Pose2d blueTwoPose = new Pose2d(2,6,Rotation2d.fromDegrees(0));
    public static final Pose2d redTwoPose = new Pose2d(Constants.FIELD_WIDTH - 2, 6, Rotation2d.fromDegrees(180));
    public static final Pose2d blueThreePose = new Pose2d(2,5,Rotation2d.fromDegrees(0));
    public static final Pose2d redThreePose = new Pose2d(Constants.FIELD_WIDTH - 2, 5, Rotation2d.fromDegrees(180));
    public static final Pose2d[] initRobotPoses = {blueOnePose, blueTwoPose, blueThreePose, redOnePose, redTwoPose, redThreePose};
  
    public static final Pose2d autoEndPose = new Pose2d(5.891426328307202, 6.045027362781998, Rotation2d.fromDegrees(90));
  }
}
