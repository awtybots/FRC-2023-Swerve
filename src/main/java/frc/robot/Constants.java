// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CustomConstants {
        public static final double rampRate = 2;
        public static final double stickDeadband = 0.1;

        public static final double lowSpeedMultiplier = 0.2;

        public static final boolean fieldRelative = true;

        public static final int LEDPort = 9;
    }

    public static final class Balance {
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 0;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    }

    public static final class Position {
        public static final class StowPosition {
            public static final int ElevatorPosition = 4000;
            public static final int ArmPosition = 0;
            public static final int ClawPosition = 0;
        }

        public static final class Nodes {
            public static final class MidNodePosition {
                public static final int ElevatorPosition = 75000;
                public static final int ArmPosition = 38;
                public static final int ClawPosition = 15;
            }

            public static final class TransitionHighNodePosition {
                public static final int ArmPosition = 35;
                public static final int ClawPosition = 0;
            }

            public static final class HighNodePosition {
                public static final int ElevatorPosition = 215000;
                public static final int ArmPosition = 58;
                public static final int ClawPosition = 16;
            }
        }

        public static final class Intake {
            public static final class IntakeConeOnSidePosition {
                public static final int ElevatorPosition = 0;
                public static final int ArmPosition = 0;
                public static final int ClawPosition = 0;
            }

            public static final class IntakeFromHumanPlayerPosition {
                public static final int ElevatorPosition = 105000;
                public static final int ArmPosition = 25;
                public static final int ClawPosition = 15;
            }

            public static final class IntakeFromGroundPosition {
                public static final int ElevatorPosition = 85000;
                public static final int ArmPosition = 88;
                public static final int ClawPosition = 6;
            }
        }
    }

    public static final class ElevatorConstants {
        public static final int kLeftElevatorMotorId = 20;
        public static final int kRightElevatorMotorId = 21;
        public static final int kElevatorEncoderId = 22;

        public static final double kMaxPercentOutput = 0.4;
        public static final double kRamp = 0.3;
        public static final double kWinchDiameter = 1.0;
        // 9:1
        public static final double kGearRatio = 1.0 / 9;

        // Heights
        public static final int initialHeight = 4000;
        public static final int minimumHeight = 2000;
        public static final int maximumHeight = 220000;
        public static final double ElevatorOffset = 0.05;

        // PID
        public static final double kP = 0.025;
        public static final double kI = 0.00;
        public static final double kD = 0.015;
        public static final double kF = 0.00;
    }

    public static final class ArmConstants {
        public static final int kLeftArmMotorId = 7;
        public static final int kRightArmMotorId = 12;

        public static final int kCurrentLimit = 40;

        // Heights
        public static final int initialHeight = 0;
        public static final int minimumHeight = 0;
        public static final int maximumHeight = 70;

        // PID
        public static final double kP = 0.001;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kF = 0.00;

        public static final double armLength = 0.54; // 0.54 m
        public static final double startingAngle = 35;

        public static final double rampRate = 0.6;
    }

    public static final class ClawConstants {
        public static final int kPivotMotorId = 6;
        public static final int kLeftIntakeMotorId = 15;
        public static final int kRightIntakeMotorId = 16;

        public static final int kClawCurrentLimit = 40;
        public static final int kIntakeCurrentLimit = 20;

        public static final int initialHeight = 0;
        public static final int minimumHeight = -15;
        public static final int maximumHeight = 35;

        public static final double kP = 0.04;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
    }

    public static final class PistonConstants {
        public static final int kTraverseSolenoidF = 0;
        public static final int kTraverseSolenoidR = 1;
    }

    public static final class LimeLightConstants {
        public static final double distanceToTarget = 4;

        public static final double AprilTagHeight = 0.945;
        public static final double LimelightHeight = 0.415;
        public static final double LimelightAngle = 14;
    }

    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        // public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        // public static final double kFrontRightChassisAngularOffset = 0;
        // public static final double kBackLeftChassisAngularOffset = Math.PI;
        // public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final double kFrontLeftChassisAngularOffset = 0;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 9;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kRearRightTurningCanId = 5;

        public static final boolean kGyroReversed = true;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction =
                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
                kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.4;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}
