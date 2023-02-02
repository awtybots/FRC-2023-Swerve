// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.PathPlannerAuto;
//import frc.robot.autos.forward;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionTracking;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.util.AutonManager;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Autonomous manager import
  private final AutonManager autonManager = new AutonManager();

  // The robot's subsystems
  private final Swerve s_Swerve = new Swerve();
  private final LimelightSubsystem Limelight = new LimelightSubsystem();

  // The driver's controller
  private final Joystick driver = new Joystick(0);

  /* Driver Buttons */
  private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton swerveSpeedToggleButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton visionTrackingToggleButton = new JoystickButton(driver, XboxController.Button.kX.value);

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void addAutonomousChoices() {
    autonManager.addOption("Do Nothing", new InstantCommand());
    autonManager.addOption("PathPlanner Test", new PathPlannerAuto(s_Swerve));
}

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.s
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis));
    swerveSpeedToggleButton.onTrue(new InstantCommand(() -> s_Swerve.toggleSwerveMode()));
    zeroGyroButton.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    visionTrackingToggleButton.whileTrue(new VisionTracking(s_Swerve, Limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonManager.getSelected();
  }
}
