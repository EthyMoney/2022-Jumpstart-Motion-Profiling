// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
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
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Get all of the paths from the pathplanner folde rand present it on the dashboard   
    // Shuffleboard Dashboard
    autoChooser.setDefaultOption("Drive Forwards", "Drive Forwards");

    try {
        // Create a file object
        File f = new File("./src/main/deploy/pathplanner");

        // Get all the names of the files present
        // in the given directory
        File[] files = f.listFiles();
        System.out.println("Files are:");
        // Display the names of the files
        for (int i = 0; i < files.length; i++) {
            String file_name = files[i].getName();
            String file_extention = file_name.substring(file_name.length() - 5, file_name.length());
            String path_name = file_name.substring(0, file_name.length() - 5);
            if (file_extention.equals(".path")){
              autoChooser.addOption(path_name, path_name);
            }
        }
    }
    catch (Exception e) {
        System.err.println(e.getMessage());
    }

    SmartDashboard.putData("Autonomous routine", autoChooser);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getRawAxis(1), -m_driverController.getRawAxis(0)),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
//     // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = autoChooser.getSelected();
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(autoName, new PathConstraints(4, 3));

    // Prints for running in simulation, you can comment these our if you want 
    System.out.print("========== Starting Auto ==========\n");
    System.out.print("Path: " + autoName + "\n");
    System.out.print("\n\n");

    /**
     *  This HashMap is for all of your manipulator commands tu run durring your autos 
     *  first argument is the command name you set in PathPlanner
     *  second argument is the command in jave you want to run, for this example we are doing a print
     */
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakeOn", new PrintCommand("Intake On"));
    eventMap.put("intakeOff", new PrintCommand("Intake Off"));
    eventMap.put("intakeUp", new PrintCommand("Intake Up"));
    eventMap.put("intakeDown", new PrintCommand("Intake Down"));
    eventMap.put("shooterOn", new PrintCommand("SHOOTING!"));
    eventMap.put("shooterOff", new PrintCommand("Shooter Off"));
    eventMap.put("wait", new PrintCommand("Waiting"));

    // Reset our robots odometry to where our aut is starting from 
    m_robotDrive.resetOdometry(examplePath.getInitialPose());

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command.
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
