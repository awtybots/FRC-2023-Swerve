// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.lib.util.AutonManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final AutonManager autonManager = new AutonManager();

  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton swerveSpeedToggleButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton visionTrackingToggleButton = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LedSubsystem s_Led = new LedSubsystem(120);
  private final LimelightSubsystem Limelight = new LimelightSubsystem(s_Led);

  public static boolean isAutoTargetOn = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    addAutonomousChoices();
    autonManager.displayChoices();
    // Configure the button bindings
    configureButtonBindings();
  }

  private void addAutonomousChoices() {
    autonManager.addOption("Do Nothing", new InstantCommand());
    autonManager.addOption("PathPlanner Test", new PathPlannerTest(s_Swerve));
    autonManager.addOption("Defaut Test", new exampleAuto(s_Swerve));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    /* Driver Buttons */
    // toggles between high speed and low speed mode
    swerveSpeedToggleButton.whenPressed(new InstantCommand(() -> s_Swerve.toggleSwerveMode()));
    zeroGyroButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

    visionTrackingToggleButton.whenHeld(new VisionTracking(s_Swerve, Limelight, fieldRelative, openLoop));
    visionTrackingToggleButton.whenReleased(new InstantCommand(() ->  isAutoTargetOn = false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonManager.getSelected();
  }
}
