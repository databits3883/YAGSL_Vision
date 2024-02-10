
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.FieldDriverStick;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.Optional;
import java.util.OptionalInt;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),Constants.ROBOT_FROGGY_CONFIG_LOCATION));
  
  //Vision, set to default no camera mode
  private static VisionSubsystem m_robotVision = new VisionSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  FieldDriverStick m_driveStick = new FieldDriverStick(m_driverController);

  JoystickButton m_calibrateButton = new JoystickButton(m_driverController, 8);

  // Build an auto chooser. This will use Commands.none() as the default option.
  SendableChooser<Command> m_autoChooser = AutoBuilder.buildAutoChooser();

  public static DriverStation.Alliance allianceColor = DriverStation.Alliance.Blue;

  public static VisionSubsystem getRobotVision() {
    return m_robotVision;
  }
  public static void setRobotVision(VisionSubsystem robotVision) {
    m_robotVision = robotVision;
  }

  public static boolean isRedAlliance() 
  {
    return allianceColor.equals(DriverStation.Alliance.Red);
  }

  public static boolean isBlueAlliance() {
    return !isRedAlliance();
  }

  public static void setAlliance(Optional<DriverStation.Alliance> color) {
    if (color.isPresent()) {
      DriverStation.Alliance currentColor = allianceColor;      
      allianceColor = color.get();
      if (!allianceColor.equals(currentColor))
      {
        System.out.println("Changed alliance to " +(isBlueAlliance()?"Blue":"Red"));
      }
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //Set up the camera
    if (Constants.VisionConstants.hasCamera)
    {
      RobotContainer.setRobotVision(new VisionSubsystem(Constants.VisionConstants.cameraY, Constants.VisionConstants.cameraX, Constants.VisionConstants.cameraZ, Constants.VisionConstants.cameraName));
    } 

    // Configure the trigger bindings
    configureBindings();
    

    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> MathUtil.applyDeadband(m_driveStick.getY(),
    //                                                                                             OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(m_driveStick.getX(),
    //                                                                                             OperatorConstants.LEFT_X_DEADBAND),
    //                                                                () -> MathUtil.applyDeadband(m_driveStick.getZ(),
    //                                                                                             OperatorConstants.RIGHT_X_DEADBAND),
    //                                                                () -> m_driverController.getRawButton(11),
    //                                                                () -> m_driverController.getRawButton(12),
    //                                                                () -> m_driverController.getRawButton(13),
    //                                                                () -> m_driverController.getRawButton(16));

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    /* Not going to use    
      Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driveStick.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());
         */


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    //Set default position to what is currently select in Drive's Station
    setInitialPose();

    drivebase.zeroGyro();
    //Add the auto chooser to the smart dashboard
    SmartDashboard.putData("Auto Chooser", m_autoChooser);    
  }

  /**
   * Set the initial pose of the robot based on driverstation selection
   */
  public void setInitialPose()
  {
    //Read position
    OptionalInt driverStationLocation = DriverStation.getLocation();
    int location = (driverStationLocation.isPresent())?driverStationLocation.getAsInt():1;
    //set zero based
    location--;

    //Set default to robot on field position
    if (isRedAlliance()) 
    {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[3+location]);
    } else {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[location]);
    }    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //Calibrate the robot button
    m_calibrateButton.onTrue((new InstantCommand(this::zeroGyroWithAlliance)));


    //Test drive to pose button
    new JoystickButton(m_driverController, 13).onTrue(drivebase.driveToPose(Constants.PoseConstants.autoEndPose));

    //Add buttons based on vision
    if (Constants.VisionConstants.hasCamera)
    {
      //Test aim at a target function, no idea if it works
      //TODO: TEST this
      new JoystickButton(m_driverController, 15).onTrue(drivebase.aimAtTarget(m_robotVision.getTarget()));
      //Debug vision button
      new JoystickButton(m_driverController, 14).onTrue(new InstantCommand(drivebase::visionPose));

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    
    return m_autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * 
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
      if (isRedAlliance())
      {
        drivebase.zeroGyro();
        //Set the pose 180 degrees
        drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else {
         drivebase.zeroGyro();      
      }
  }
  
}
