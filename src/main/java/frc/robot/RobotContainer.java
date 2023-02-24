// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.PathPlannerAuto;
import frc.robot.commands.Autonomous.AutomatedVisionTracking;
import frc.robot.commands.Autonomous.Balance;
import frc.robot.commands.DriveParts.*;
import frc.robot.commands.Positions.Intake.IntakeFromGroundPosition;
import frc.robot.commands.Positions.Intake.IntakeFromHumanPlayerPosition;
import frc.robot.commands.Positions.Nodes.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
// TODO: LED | import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.*;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.AutonManager;
import frc.util.Controller;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Autonomous manager import
    private final AutonManager autonManager = new AutonManager();

    // The robot's subsystems
    private final Swerve s_Swerve = new Swerve();
    // TODO: LED | private final LedSubsystem s_Led = new LedSubsystem(120);
    // TODO: LED | private final LimelightSubsystem Limelight = new LimelightSubsystem(s_Led);
    private final LimelightSubsystem Limelight = new LimelightSubsystem();

    private final ElevatorSubsystem Elevator = new ElevatorSubsystem();
    private final ArmSubsystem Arm = new ArmSubsystem(Elevator);
    private final ClawSubsystem Claw = new ClawSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem();
    private final PistonSubsystem Piston = new PistonSubsystem();

    // The driver's controller
    private final Controller driver = new Controller(0);
    private final Controller operator = new Controller(1);

    public static boolean isAutoTargetOn = false;

    // private final PathPlannerTrajectory test1Trajectory =
    //         PathPlanner.loadPath("Straight", new PathConstraints(6, 4));
    // ! Delete the above if autoBuilder works
    private final HashMap<String, Command> test1EventMap = new HashMap<>();

    public final SwerveAutoBuilder autoBuilder =
            new SwerveAutoBuilder(
                    // Pose2d supplier
                    s_Swerve::getPose,
                    // Pose2d consumer, used to reset odometry at the beginning of auto
                    s_Swerve::resetOdometry,
                    // SwerveDriveKinematics
                    Constants.DriveConstants.kDriveKinematics,
                    // PID constants to correct for translation error (used to create the X and Y PID
                    // controllers)
                    new PIDConstants(Constants.AutoConstants.kPXYController, 0.0, 0.0),
                    // PID constants to correct for rotation error (used to create the rotation controller)
                    new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
                    // Module states consumer used to output to the drive subsystem
                    s_Swerve::setModuleStates,
                    test1EventMap,
                    // TODO: Change the useAllianceColor to true? Not sure how it detects the color.
                    // Should the path be automatically mirrored depending on alliance color. Optional,
                    // defaults to true
                    false,
                    s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        eventMaps();
        addAutonomousChoices();
        autonManager.displayChoices();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void eventMaps() {
        // test1EventMap.put("event", new StowPosition(Elevator, Arm, Claw));
        // test1EventMap.put("stopEvent", new Balance(s_Swerve));
    }

    private void addAutonomousChoices() {
        autonManager.addOption("Do Nothing.", new InstantCommand());
        // autonManager.addOption(
        //         "PathPlanner Test1",
        //         autoBuilder.fullAuto(
        //                 PathPlanner.loadPathGroup(
        //                         "Test1",
        //                         new PathConstraints(
        //                                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))));
        // autonManager.addOption(
        //         "PathPlanner Test1Red",
        //         autoBuilder.fullAuto(
        //                 PathPlanner.loadPathGroup(
        //                         "Test1Red",
        //                         new PathConstraints(
        //                                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))));
        autonManager.addOption(
                "PathPlanner Straight",
                autoBuilder.fullAuto(
                        PathPlanner.loadPathGroup(
                                "Straight",
                                new PathConstraints(
                                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        final int translationAxis = XboxController.Axis.kLeftY.value;
        final int strafeAxis = XboxController.Axis.kLeftX.value;
        final int rotationAxis = XboxController.Axis.kRightX.value;

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis));

        driver.buttonA.onTrue(new InstantCommand(s_Swerve::toggleSwerveMode));
        driver.buttonY.onTrue(new InstantCommand(s_Swerve::zeroGyro));
        driver.buttonB.onTrue(new Balance(s_Swerve));
        driver.buttonX.onTrue(new AutomatedVisionTracking(s_Swerve, Limelight));

        Elevator.setDefaultCommand(new DriveElevator(operator, Elevator));
        Arm.setDefaultCommand(new RotateArm(operator, Arm));
        Claw.setDefaultCommand(new DriveClaw(operator, Claw));
        // Intake.setDefaultCommand(new setIntake(operator, Intake));
        Piston.setDefaultCommand(new ToggleIntakeMode(operator, Piston));

        operator.buttonA.onTrue(new StowPosition(Elevator, Arm, Claw));
        driver.rightBumper.onTrue(new StowPosition(Elevator, Arm, Claw));
        operator.buttonB.onTrue(new MidNodePosition(Elevator, Arm, Claw));
        operator.buttonY.onTrue(new HighNodePosition(Elevator, Arm, Claw));

        operator.dPadDown.onTrue(new IntakeFromGroundPosition(Elevator, Arm, Claw));
        operator.dPadUp.onTrue(new IntakeFromHumanPlayerPosition(Elevator, Arm, Claw));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autonManager.getSelected();
        // return autoBuilder.fullAuto(
        //     PathPlanner.loadPathGroup(
        //             "Straight",
        //             new PathConstraints(
        //                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)));
        return new PathPlannerAuto(
        PathPlanner.loadPath("Straight", new PathConstraints(6, 4)), s_Swerve, test1EventMap);
    }
}
