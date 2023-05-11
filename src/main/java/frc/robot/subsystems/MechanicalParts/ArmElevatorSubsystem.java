package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmElevator;
import frc.robot.Constants.Presets;
import frc.robot.RobotContainer;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ArmElevatorSubsystem extends SubsystemBase implements ArmElevatorMech {

    private CANSparkMax mArmMotor;

    private final RelativeEncoder mArmEncoder;

    private final SparkMaxPIDController mArmPIDController;

    private double armExtent;
    private final double kArmGearRatio = (1.0 / 9.0) / (48.0 / 34.0);
    private final double kDiameter = 1.5;

    public ArmElevatorSubsystem() {

        mArmMotor = new CANSparkMax(ArmElevator.kArmMotorId, MotorType.kBrushless);
        mArmMotor.restoreFactoryDefaults();

        // Current limit
        mArmMotor.setSmartCurrentLimit(ArmElevator.kCurrentLimit);

        armExtent = ArmElevator.initialExtent;

        mArmEncoder = mArmMotor.getEncoder();

        mArmPIDController = mArmMotor.getPIDController();

        // mArmPIDController.setP(ArmElevator.kP);
        // mArmPIDController.setI(ArmElevator.kI);
        // mArmPIDController.setD(ArmElevator.kD);
        mArmPIDController.setP(0.08);
        mArmPIDController.setI(0);
        mArmPIDController.setD(0);

        mArmPIDController.setOutputRange(-0.6, 0.6);

        // mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setExtent(double value) {
        armExtent = value;
    }

    public void setExtentInches(double inches) {
        double sensorUnits =
                Convert.distanceToEncoderPos(inches, kArmGearRatio, kDiameter, Encoder.RevRelativeEncoder);
        setExtent(sensorUnits);
    }

    public double getExtent() {
        final double rawRevs = mArmEncoder.getPosition();
        System.out.println(rawRevs);
        final double extent =
                Convert.encoderPosToDistance(rawRevs, kArmGearRatio, kDiameter, Encoder.RevRelativeEncoder);
        System.out.println(extent);
        return extent;
    }

    public void resetEncoderValue() {
        armExtent = 0;
        mArmEncoder.setPosition(armExtent);
    }

    public void drive(double pct) {
        if (!RobotContainer.getResetPosMode()) {
            armExtent = MathUtil.clamp(armExtent, ArmElevator.minimumExtent, ArmElevator.maximumExtend);
        }
        // MathUtil.clamp(armHeight, ArmConstants.minimumHeight, getMaximumRotation());
        armExtent += pct;
    }

    public boolean atTargetExtent() {
        return Math.abs(getTickError()) < Presets.ArmThreshold;
    }

    public double getTickError() {
        return mArmEncoder.getPosition() - armExtent;
    }

    @Override
    public void periodic() {
        mArmPIDController.setReference(armExtent, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber(
                "Arm Error",
                Convert.encoderPosToDistance(
                        mArmEncoder.getPosition() - armExtent,
                        kArmGearRatio,
                        kDiameter,
                        Encoder.RevRelativeEncoder));
        SmartDashboard.putNumber("Arm extent", mArmEncoder.getPosition());
        SmartDashboard.putNumber("Arm Target Extent", this.armExtent);
    }

    public void stop() {
        mArmMotor.set(0);
    }
}
