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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  
  //Vision
  public static VisionSubsystem robotVision = new VisionSubsystem(Constants.VisionConstants.cameraY, Constants.VisionConstants.cameraX, Constants.VisionConstants.cameraZ, Constants.VisionConstants.cameraName);

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  FieldDriverStick m_driveStick = new FieldDriverStick(m_driverController);

  JoystickButton m_calibrateButton = new JoystickButton(m_driverController, 8);

  //Face forward
  Pose2d blueOnePose = new Pose2d(2,7,Rotation2d.fromDegrees(0));
  Pose2d redOnePose = new Pose2d(10, 7, Rotation2d.fromDegrees(180));

  //Face Right, move diagonal
  Pose2d defaultZeroPosition = new Pose2d(0.33 ,0.33,Rotation2d.fromDegrees(0));

  private final Command m_rev180 = drivebase.getAutonomousCommand("rev180", true);
  //private final Command m_testPath = drivebase.getAutonomousCommand("Test Path", true);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static DriverStation.Alliance allianceColor = DriverStation.Alliance.Blue;

  //XboxController driverXbox = new XboxController(0);

  public static boolean isRedAlliance() {
    return allianceColor == DriverStation.Alliance.Red;
  }

  public static boolean isBlueAlliance() {
    return allianceColor == DriverStation.Alliance.Blue;
  }

  public static void setAlliance(Optional<DriverStation.Alliance> color) {
    if (color.isPresent()) {
      DriverStation.Alliance currentColor = allianceColor;      
      allianceColor = color.get();
      if (allianceColor != currentColor)
        System.out.println("Changed colors to " +(isBlueAlliance()?"Blue":"Red"));
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
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

        
    
    //zero gyro, set offset 90 degrees
    //drivebase.setGyroOffset(Math.toRadians(-90));

    drivebase.zeroGyro();


    //Set default to robot on field position
    //drivebase.resetOdometry(defaultZeroPosition);
    //drivebase.resetOdometry(defaultFaceForwardPose);
    if (isRedAlliance()) {
      drivebase.resetOdometry(redOnePose);
    } else {
      drivebase.resetOdometry(blueOnePose);
    }

    //m_chooser.setDefaultOption("Test Path", m_testPath);
    m_chooser.setDefaultOption("rev180", m_rev180);

    SmartDashboard.putData(m_chooser);

    
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_calibrateButton.onTrue((new InstantCommand(drivebase::zeroGyro)));
    new JoystickButton(m_driverController, 14).onTrue(new InstantCommand(robotVision::debugClosestTarget));
    //new JoystickButton(m_driverController, 11).onTrue(new InstantCommand));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    
    return m_chooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
