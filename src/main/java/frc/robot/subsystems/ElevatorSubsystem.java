package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;
    private final double kWinchDiameter;
    private final double kGearRatio;

    private TalonFX mLeftElevatorMotor;
    private TalonFX mRightElevatorMotor;
    private final TalonFX[] motors;

    public ElevatorSubsystem() {
        kMaxPercentOutput = Constants.TalonConstants.kMaxPercentOutput;
        kRamp = Constants.TalonConstants.kRamp;
        kWinchDiameter = Constants.TalonConstants.kWinchDiameter;
        kGearRatio = Constants.TalonConstants.kGearRatio;

        mLeftElevatorMotor = new TalonFX(Constants.TalonConstants.kLeftElevatorMotorId);
        mRightElevatorMotor = new TalonFX(Constants.TalonConstants.kRightElevatorMotorId);
        motors = new TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        configMotors();
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();

        mLeftElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        for (TalonFX motor : motors) {
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            motor.setSelectedSensorPosition(0.0);

            motor.setNeutralMode(NeutralMode.Brake);

            motor.configOpenloopRamp(kRamp);
            motor.configClosedloopRamp(kRamp);
            motor.configPeakOutputForward(kMaxPercentOutput);
            motor.configPeakOutputReverse(-kMaxPercentOutput);
            motor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
        }
    }

    public void drive(double pct) {
        for (TalonFX motor : motors)
            motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
    }

    public void stop() {
        for (TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
    }

    private double getPosition() {
        double sum = 0.0;
        for (TalonFX motor : motors) {
            sum +=
                    Convert.encoderPosToDistance(
                            motor.getSelectedSensorPosition(),
                            kGearRatio,
                            kWinchDiameter,
                            Encoder.TalonFXIntegrated);
        }
        return sum / motors.length;
    }

}