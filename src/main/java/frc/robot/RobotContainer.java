// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.Diagnostic;
import frc.robot.commands.Autonomous.Balance;
import frc.robot.commands.Autonomous.Pickup;
import frc.robot.commands.Autonomous.Place;
import frc.robot.commands.DriveParts.*;
import frc.robot.commands.Positions.Intake.IntakeFromGroundLowPosition;
import frc.robot.commands.Positions.Intake.IntakeFromGroundPosition;
import frc.robot.commands.Positions.Intake.IntakeFromHumanPlayerPosition;
import frc.robot.commands.Positions.Intake.IntakeFromSlidingHumanPlayerPosition;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.*;
import frc.robot.subsystems.Swerve.Swerve;
// import frc.robot.subsystems.ledutils;
// import frc.robot.subsystems.ledutils.patterens_eneum;
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
    private final AutonManager autonManager;

    // The robot's subsystems
    private final Swerve s_Swerve = new Swerve();
    // ! private final LedSubsystem s_Led = new LedSubsystem(0, 120);
    private final LimelightSubsystem Limelight = new LimelightSubsystem();

    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final ClawSubsystem s_Claw = new ClawSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();

    private static Boolean isCone = Constants.CustomConstants.isCone;
    private static Boolean resetPosMode = false;
    private static double angleOffset = 0;

    // The controllers
    private final Controller driverController = new Controller(0);
    private final Controller operatorController = new Controller(1);

    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final String[] autonChoices =
            new String[] {
                "LeftPlaceBalance",
                "LeftPlacePickup",
                "LeftPlacePickupBalance",
                "LeftPlacePickupPlace",
                "LeftPlacePickupPlaceBalance",
                "MiddlePlaceBalance",
                "RightPlaceBalance",
                "RightPlacePickup",
                "RightPlacePickupBalance",
                "RightPlacePickupPlace",
                "RightPlacePickupPlaceBalance"
            };

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
                    eventMap,
                    // TODO: make sure that the drive team understands that the alliance color thing matters
                    true,
                    s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autonManager = new AutonManager(Limelight);
        eventAssignment();
        addAutonomousChoices();
        autonManager.displayChoices();
        // Configure the button bindings
        configureButtonBindings();
        // s_Led.ivans_patterns(patterens_eneum.awtybots);
        SmartDashboard.putBoolean("EmergencyButton", false);
    }

    /**
     * Use this method to define the command or command groups to be run at each event marker key. New
     * event markers can be created in PathPlanner.
     */
    private void eventAssignment() {
        eventMap.put("Pickup", new Pickup(s_Claw, s_Arm, s_Elevator, s_Intake));
        eventMap.put(
                "Place", new Place(s_Swerve, Limelight, s_Claw, s_Arm, s_Elevator, s_Intake, 0, false));
        eventMap.put(
                "PlaceHigh", new Place(s_Swerve, Limelight, s_Claw, s_Arm, s_Elevator, s_Intake, 1, false));
        eventMap.put("Balance", new Balance(s_Swerve));
    }
    // The RightPlacePickupPlaceBalance is : 1 foot from DriverStation blue line (x: 2.16), 6 inches
    // from Right wall (y: 0.76).
    // The
    /** Use this method to add Autonomous paths, displayed with {@link AutonManager} */
    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Do Nothing.", new InstantCommand());
        autonManager.addOption(
                "Diagnostic",
                new Diagnostic(s_Elevator, s_Arm, s_Claw, s_Intake, s_Swerve).withTimeout(1.5));
        for (var i = 0; i < autonChoices.length; i++) {
            autonManager.addOption(
                    autonChoices[i],
                    autoBuilder.fullAuto(
                            PathPlanner.loadPathGroup(
                                    autonChoices[i],
                                    new PathConstraints(
                                            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))));
        }
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared))));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    public void setIsCone(boolean value) {
        isCone = value;
    }

    public static boolean getIsCone() {
        return isCone;
    }

    public static boolean getResetPosMode() {
        return resetPosMode;
    }

    public static void setResetPosMode(boolean mode) {
        resetPosMode = mode;
    }

    public static double getAngleOffset() {
        return angleOffset;
    }

    public static void setAngleOffset(double offset) {
        angleOffset = offset;
    }

    public void autonResetGyro() {
        s_Swerve.zeroGyro(180);
    }

    private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        final int translationAxis = XboxController.Axis.kLeftY.value;
        final int strafeAxis = XboxController.Axis.kLeftX.value;
        final int rotationAxis = XboxController.Axis.kRightX.value;

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve, driverController, translationAxis, strafeAxis, rotationAxis));

        driverController.buttonA.onTrue(new InstantCommand(s_Swerve::toggleSwerveMode));
        driverController.buttonY.onTrue(new InstantCommand(s_Swerve::zeroGyro));
        // ! driverController.buttonB.onTrue(new Balance(s_Swerve));
        // ! driverController.buttonX.onTrue(new VisionTracking(s_Swerve, Limelight));

        // April Tag Mode
        driverController.leftTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(1);
                            setIsCone(false);
                            // s_Led.ivans_patterns(patterens_eneum.cube);
                            ;
                        }));

        // Reflective Tape Mode
        driverController.rightTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(3);
                            setIsCone(true);
                            // s_Led.ivans_patterns(patterens_eneum.cone);
                            ;
                        }));

        s_Elevator.setDefaultCommand(new DriveElevator(operatorController, s_Elevator));
        s_Arm.setDefaultCommand(new RotateArm(operatorController, s_Arm));
        s_Claw.setDefaultCommand(new DriveClaw(operatorController, s_Claw));
        s_Intake.setDefaultCommand(new setIntake(operatorController, s_Intake));

        operatorController.buttonA.onTrue(new StowPosition(s_Elevator, s_Arm, s_Claw));
        driverController.rightBumper.onTrue(new StowPosition(s_Elevator, s_Arm, s_Claw));
        operatorController.buttonB.onTrue(new MidNodePosition(s_Elevator, s_Arm, s_Claw));
        operatorController.buttonY.onTrue(new HighNodePosition(s_Elevator, s_Arm, s_Claw));
        // operatorController.buttonY.onTrue(new Place(s_Swerve, Limelight, s_Claw,s_Arm, s_Elevator,
        // s_Intake, 1, true ));

        // Emergency mode
        operatorController.buttonBack.onTrue(
                new InstantCommand(
                        () -> {
                            if (getResetPosMode()) {
                                s_Elevator.resetEncoderValue();
                                s_Arm.resetEncoderValue();
                                s_Claw.resetEncoderValue();
                            }
                            ;
                        }));

        // operatorController.dPadDown.onTrue(new IntakeFromGroundPosition(s_Elevator, s_Arm, s_Claw));
        operatorController.dPadDown.onTrue(new IntakeFromGroundPosition(s_Elevator, s_Arm, s_Claw));
        operatorController.dPadUp.onTrue(new IntakeFromHumanPlayerPosition(s_Elevator, s_Arm, s_Claw));
        operatorController.dPadRight.onTrue(
                new IntakeFromSlidingHumanPlayerPosition(s_Elevator, s_Arm, s_Claw));
        operatorController.dPadLeft.onTrue(new IntakeFromGroundLowPosition(s_Elevator, s_Arm, s_Claw));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // SendableChooser<InstantCommand> isConeChooser = new SendableChooser<InstantCommand>();
        // isConeChooser.setDefaultOption("Cube", new InstantCommand(() -> Limelight.setPipeline(0)));
        // isConeChooser.addOption("Cone", new InstantCommand(() -> Limelight.setPipeline(1)));
        // SmartDashboard.putData("PipelineChooser", isConeChooser);
        // final String isConeDashboardSelection =
        //         NetworkTableInstance.getDefault()
        //                 .getTable("SmartDashboard")
        //                 .getSubTable("PipelineChooser")
        //                 .getEntry("active")
        //                 .getString("Cube");
        // Limelight.setPipeline(isConeDashboardSelection == "Cube" ? 0 : 1);
        // // setIsCone(isConeDashboardSelection != "Cube");
        // setIsCone(false);
        // return autonManager.getSelected();
        return autoBuilder.fullAuto(
                PathPlanner.loadPathGroup("GyroTest", new PathConstraints(1, 0.5))); // ! Test
    }

    public void idleLimelight() {
        Limelight.setPipeline(7);
        Limelight.setMode(0); // pipeline default
    }
}
