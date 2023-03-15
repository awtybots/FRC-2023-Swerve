package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Presets;
import frc.robot.RobotContainer;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    public final RelativeEncoder mRightArmEncoder;

    private final SparkMaxPIDController mRightArmPIDController;

    public double armHeight;
    private final double kArmGearRatio = 1.0 / (216.0);

    public ArmSubsystem() {

        mLeftArmMotor = new CANSparkMax(Arm.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Arm.kLeftArmMotorId, MotorType.kBrushless);
        // mLeftArmMotor.restoreFactoryDefaults();

        mRightArmMotor.restoreFactoryDefaults();

        // Current limit
        mLeftArmMotor.setSmartCurrentLimit(Arm.kCurrentLimit);
        mRightArmMotor.setSmartCurrentLimit(Arm.kCurrentLimit);

        mRightArmMotor.setInverted(true);
        mLeftArmMotor.follow(mRightArmMotor, true);

        armHeight = Arm.initialHeight;

        mRightArmEncoder = mRightArmMotor.getEncoder();

        mRightArmPIDController = mRightArmMotor.getPIDController();

        mRightArmPIDController.setP(Arm.kP);
        mRightArmPIDController.setI(Arm.kI);
        mRightArmPIDController.setD(Arm.kD);
        mRightArmPIDController.setOutputRange(-0.5, 0.5);

        // mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(double value) {
        armHeight = value;
    }

    public void setDegrees(double degrees) {
        double encoderunits =
                Convert.angleToEncoderPos(degrees, kArmGearRatio, Encoder.RevRelativeEncoder);
        armHeight = encoderunits;
    }

    public double getAngle() {
        final double rawRevs = mRightArmEncoder.getPosition();
        final double theta =
                Convert.encoderPosToAngle(rawRevs, kArmGearRatio, Encoder.RevRelativeEncoder);
        return Arm.startingAngle - theta;
    }

    public void resetEncoderValue() {
        armHeight = 0;
        mRightArmEncoder.setPosition(armHeight);
    }

    public void drive(double pct) {
        if (!RobotContainer.getResetPosMode()) {
            armHeight = MathUtil.clamp(armHeight, Arm.minimumHeight, 100);
        }
        // MathUtil.clamp(armHeight, ArmConstants.minimumHeight, getMaximumRotation());
        armHeight += pct * Arm.armConversion;
    }

    public boolean isFinished() {
        return Math.abs(mRightArmEncoder.getPosition() - armHeight) < Presets.ArmThreshold;
    }

    @Override
    public void periodic() {
        mRightArmPIDController.setReference(
                armHeight,
                CANSparkMax.ControlType.kPosition,
                0,
                Arm.arbitraryFeedFowardRate * Math.cos(getAngle()));
        SmartDashboard.putNumber(
                "Arm Error",
                Convert.encoderPosToAngle(
                        mRightArmEncoder.getPosition() - armHeight, kArmGearRatio, Encoder.RevRelativeEncoder));
        SmartDashboard.putNumber("Arm angle", this.getAngle());
    }

    public void stop() {
        mRightArmMotor.set(0);
    }
}
