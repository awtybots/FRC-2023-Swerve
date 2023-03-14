package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    public final RelativeEncoder mRightArmEncoder;

    private final SparkMaxPIDController mRightArmPIDController;

    public double armHeight;
    private final double kArmGearRatio = 1.0 / 216.0;

    public ArmSubsystem() {

        mLeftArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
        // mLeftArmMotor.restoreFactoryDefaults();

        mRightArmMotor.restoreFactoryDefaults();

        // Current limit
        mLeftArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
        mRightArmMotor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);

        mRightArmMotor.setInverted(true);
        mLeftArmMotor.follow(mRightArmMotor, true);

        armHeight = Constants.ArmConstants.initialHeight;

        mRightArmEncoder = mRightArmMotor.getEncoder();

        mRightArmPIDController = mRightArmMotor.getPIDController();

        mRightArmPIDController.setP(0.04);
        mRightArmPIDController.setI(0);
        mRightArmPIDController.setD(0);
        mRightArmPIDController.setOutputRange(-0.5, 0.5);

        // mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(double value) {
        armHeight = value;
    }

    public double getAngle() {
        final double rawRevs = mRightArmEncoder.getPosition();
        final double theta =
                Convert.encoderPosToAngle(rawRevs, kArmGearRatio, Encoder.RevRelativeEncoder);
        System.out.println(theta);
        return Constants.ArmConstants.startingAngle - theta;
    }

    public void resetEncoderValue() {
        armHeight = 0;
        mRightArmEncoder.setPosition(armHeight);
    }

    public void drive(double pct) {
        if (!RobotContainer.getResetPosMode()) {
            armHeight = MathUtil.clamp(armHeight, Constants.ArmConstants.minimumHeight, 100);
        }
        // MathUtil.clamp(armHeight, Constants.ArmConstants.minimumHeight, getMaximumRotation());
        armHeight += pct * Constants.ArmConstants.armConversion;
    }

    public boolean isFinished() {
        return Math.abs(mRightArmEncoder.getPosition() - armHeight) < Constants.Position.ArmThreshold;
    }

    @Override
    public void periodic() {
        mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition, 0, Constants.ArmConstants.arbitraryFeedFowardRate * Math.cos(getAngle()));
        SmartDashboard.putNumber("Arm encoder readout", mRightArmEncoder.getPosition());
        SmartDashboard.putNumber("Arm angle", this.getAngle());
    }

    public void stop() {
        mRightArmMotor.set(0);
    }
}
