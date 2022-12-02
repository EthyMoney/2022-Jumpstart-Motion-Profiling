// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    // m_robotDrive.setDefaultCommand(
    //     // A split-stick arcade command, with forward/backward controlled by the left
    //     // hand, and turning controlled by the right.
    //     new RunCommand(
    //         () ->
    //             m_robotDrive.arcadeDrive(
    //                 -m_driverController.getLeftY(), m_driverController.getRightX()),
    //         m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
//   private void configureButtonBindings() {
//     // Drive at half speed when the right bumper is held
//     new JoystickButton(m_driverController, Button.kRightBumper.value)
//         .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
//         .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
//   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String robot_path = "Example Path"; // replace with auto selector 
    System.out.print("========== Starting Auto ==========\n");
    System.out.print("Path: " + robot_path + "\n");
    System.out.print("\n\n");
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(robot_path, new PathConstraints(4, 3));
    HashMap<String, Command> eventMap = new HashMap<>();
    
    eventMap.put("intakeOn", new PrintCommand("Intake On"));
    eventMap.put("intakeOff", new PrintCommand("Intake Off"));
    eventMap.put("intakeUp", new PrintCommand("Intake Up"));
    eventMap.put("intakeDown", new PrintCommand("Intake Down"));
    eventMap.put("wait", new PrintCommand("Waiting"));

    m_robotDrive.resetOdometry(examplePath.getInitialPose());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        m_robotDrive::getPose, // Pose2d supplier
        m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        m_robotDrive::tankDriveVolts, //BiConsumer<Double, Double> outputMetersPerSecond,
        eventMap,
        m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
    );

    Command fullAuto = autoBuilder.fullAuto(examplePath);

    // Run path following command, then stop at the end.
    return fullAuto.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
