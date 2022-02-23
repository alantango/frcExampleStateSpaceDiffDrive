// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // set up possible auto start position
    m_chooser.setDefaultOption("None", null);
    m_chooser.addOption("T1-Left","T1-Left");
    m_chooser.addOption("T1-Center","T1-Center");
    m_chooser.addOption("T1-Right","T1-Right");
    m_chooser.addOption("T2-Left", "T2-Left");
    m_chooser.addOption("T2-Center","T2-Center");
    m_chooser.addOption("T2-Right", "T2-Right");
    SmartDashboard.putData(m_chooser);

    //init delay of auto run
    SmartDashboard.putNumber("Auto Init Delay: ", 1);
    SmartDashboard.putNumber("Auto Command Pause: ", 0.5);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), m_driverController.getRightX()*.55),
            m_robotDrive));
  }

  public SendableChooser<String> autoStartChooser(){
    return m_chooser;
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

  public Command getAutonomousCommand(){
    
    m_robotDrive.zeroHeading();
    m_robotDrive.resetCurrPositionHeading();

    return getSimpleAutoCommand();
    // return getRamesetAutoCommand();
  }

  public Command getSimpleAutoCommand(){
    
    Command cmd = null;

    double ax, ay, ah;
    String position = m_chooser.getSelected();
    if(position==null)
      return null;

    switch(position){
      case "T1-Left":
        ax=6.5; ay=5.5; ah=-44;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.65)).andThen(
          maneuver(0, 0.23, 0.5)).andThen(
          maneuver(0.4, -0.20, 2.68)).andThen(
            fullStop());
        break;
      case "T1-Center":
        ax=6.1; ay=4.95; ah=-21;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.75)).andThen(
          maneuver(0.5, 0, 1.38)).andThen(
            fullStop());
        break;
      case "T1-Right":
        ax=6; ay=4.13; ah=0;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.7)).andThen(
          maneuver(0, -0.239, 0.55)).andThen(
          maneuver(0.4, 0.20, 2.78)).andThen(
          fullStop());
          break;
      case "T2-Left":
        ax=6.85; ay=2.35; ah=44;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.65)).andThen(
          maneuver(0, 0.23, 0.5)).andThen(
          maneuver(0.4, -0.20, 2.68)).andThen(
            fullStop());
          break;
      case "T2-Center":
        ax=7.45; ay=1.97; ah=70;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.75)).andThen(
          maneuver(0.5, 0, 1.4)).andThen(
            fullStop());
        break;
      case "T2-Right":
        ax=8.2; ay=1.85; ah=90;
        cmd = initWait().andThen(
          maneuver(-0.5, 0, 0.65)).andThen(
          maneuver(0, -0.25, 0.4)).andThen(
          maneuver(0.4, 0.19, 2.5)).andThen(
          fullStop());
        break;
      default:
        return null;
    }
    // ah is heading expressed in 360, gyro degree range is -180/180
    m_robotDrive.resetOdometry(new Pose2d(ax, ay, Rotation2d.fromDegrees(ah/2)));  
    return cmd;
    
  }

  private Command maneuver(double fwd, double rot, double elapse){
    return new RunCommand(
      () -> m_robotDrive.arcadeDrive(fwd, rot), m_robotDrive)
      .withTimeout(elapse)
      .andThen(pause());
  }

  private Command fullStop(){
    return maneuver(0,0,0);
  }

  private Command pause(double seconds){
    return new WaitCommand(seconds);
  }

  private Command initWait(){
    double s = SmartDashboard.getNumber("Auto Init Delay: ", 1);
    return pause(s);
  }
  
  private Command pause(){
    double s = SmartDashboard.getNumber("Auto Command Pause: ", 1);
    return pause(s);
  }
  
/* 
  public Command getRamesetAutoCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            7);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at (1, 2) facing the +X direction
            new Pose2d(6.18, 3.975, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            // List.of(new Translation2d(6.4, 4.05), new Translation2d(6.7, 4.15)),
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6.981, 4.423, new Rotation2d(-0.349)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to starting pose of trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
 */
}
