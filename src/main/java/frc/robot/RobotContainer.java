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
    m_chooser.setDefaultOption("Left-Center","Left-Center"); //
    m_chooser.addOption("Right-Center","Right-Center");
    m_chooser.addOption("Left-Up","Left-Up");
    m_chooser.addOption("Left-Down","Left-Down");
    m_chooser.addOption("Right-Up", "Right-Up");
    m_chooser.addOption("Right-Down", "Right-Down");
    SmartDashboard.putData(m_chooser);

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
                    -m_driverController.getLeftY(), m_driverController.getRightX()*.35),
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
    return getDummyAutoCommand();
    // return getRamesetAutoCommand();
  }

  public Command getDummyAutoCommand(){
    String autoStartPos = m_chooser.getSelected();
    
    m_robotDrive.zeroHeading();
    
    m_robotDrive.resetOdometry(new Pose2d(6, 4, Rotation2d.fromDegrees(0)));

    return new WaitCommand(2).andThen(
      new RunCommand(() -> m_robotDrive.arcadeDrive(-0.5, 0), m_robotDrive).withTimeout(0.69)).andThen(
      new WaitCommand(0.2)).andThen(
      new RunCommand(() -> m_robotDrive.arcadeDrive(0, -0.25), m_robotDrive).withTimeout(0.5)).andThen(
        new WaitCommand(0.2)).andThen(
      new RunCommand(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive).withTimeout(0.7)).andThen(
        new WaitCommand(0.2)).andThen(
      new RunCommand( ()-> m_robotDrive.arcadeDrive(0, 0.25), m_robotDrive).withTimeout(0.875)).andThen(
        new WaitCommand(0.2)).andThen(
      new RunCommand(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive).withTimeout(0.85)).andThen(
        new RunCommand(() -> m_robotDrive.tankDriveVolts(0,0))
      );
        //   );

    // return new WaitCommand(0.5).andThen(
    //   new RunCommand(
    //       () -> m_robotDrive.arcadeDrive(-0.5, 0), m_robotDrive))
    //    .withTimeout(2).andThen(
    //     new WaitCommand(0.5)).andThen( 
    //     new RunCommand(
    //           () -> m_robotDrive.arcadeDrive(0, -0.5), m_robotDrive).withTimeout(0.5)).andThen(
    //     new RunCommand(
    //           () -> m_robotDrive.arcadeDrive(0, 0.5), m_robotDrive).withTimeout(0.5)).andThen(
    //       ()->m_robotDrive.arcadeDrive(0.5,0)
    //     );
      
  }

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
}
