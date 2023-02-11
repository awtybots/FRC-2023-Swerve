package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;
    // private final AbsoluteEncoder mLeftArmEncoder;
    // private final AbsoluteEncoder mRightArmEncoder;
    private final CANSparkMax[] motors;

    public ArmSubsystem() {
        mLeftArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
        motors = new CANSparkMax[] {mLeftArmMotor, mRightArmMotor};
    }

    public void drive(double pct) {
        // for (CANSparkMax motor : motors) {}
        //     motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
    }

    public void stop() {
        // for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
    }

    // private double getPosition() {
    //     double sum = 0.0;
    //     for (WPI_TalonFX motor : motors) {
    //         sum +=
    //                 Convert.encoderPosToDistance(
    //                         motor.getSelectedSensorPosition(),
    //                         kGearRatio,
    //                         kWinchDiameter,
    //                         Encoder.TalonFXIntegrated);
    //     }
    //     return sum / motors.length;
    // }

}