package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.util.math.TalonConversions;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;
    private final double kWinchDiameter;
    private final double kGearRatio;

    private final WPI_TalonFX mLeftElevatorMotor;
    private final WPI_TalonFX mRightElevatorMotor;
    private final WPI_TalonFX[] motors;

    private final WPI_TalonSRX elevatorEncoder;

    public double elevatorTargetHeight = 4000;
    
    
    public ElevatorSubsystem() {
        kMaxPercentOutput = Constants.ElevatorConstants.kMaxPercentOutput;
        kRamp = Constants.ElevatorConstants.kRamp;
        kWinchDiameter = Constants.ElevatorConstants.kWinchDiameter;
        kGearRatio = Constants.ElevatorConstants.kGearRatio;
        
        elevatorEncoder = new WPI_TalonSRX(Constants.ElevatorConstants.kElevatorEncoderId); 
        configElevatorEncoder();

        mLeftElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        mRightElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        configMotors();
    }
    
    private void resetToAbsolute(){
        //double absolutePosition = TalonConversions.degreesToFalcon(getCanCoder().getDegrees(), Constants.ElevatorConstants.kGearRatio);
        double absolutePosition = (getCanCoder());
        mLeftElevatorMotor.setSelectedSensorPosition(absolutePosition);
        mRightElevatorMotor.setSelectedSensorPosition(absolutePosition);
    }

    public double getCanCoder(){
        return elevatorEncoder.getSelectedSensorPosition();
    }

    private void configElevatorEncoder(){        
        elevatorEncoder.configFactoryDefault();
        elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        // elevatorEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();

        mRightElevatorMotor.setInverted(TalonFXInvertType.CounterClockwise);

        for (WPI_TalonFX motor : motors) {
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            //motor.setSelectedSensorPosition(0.0);

            motor.setNeutralMode(NeutralMode.Brake);


            
            motor.configOpenloopRamp(kRamp+0.1);
            motor.configClosedloopRamp(kRamp);
            motor.configPeakOutputForward(kMaxPercentOutput);
            motor.configPeakOutputReverse(-kMaxPercentOutput);
            motor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
        }

        resetToAbsolute();
    }

    public void drive(double pct) {
        
        motors[0].set(ControlMode.Position, 4000);
        motors[1].set(ControlMode.Position, 4000);//TODO ELEVATOR TARGET HEIGHT
    }

    public void stop() {
        for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
    }

    private double getPosition() {
        double sum = 0.0;
        for (WPI_TalonFX motor : motors) {
            sum +=
                    Convert.encoderPosToDistance(
                            motor.getSelectedSensorPosition(),
                            kGearRatio,
                            kWinchDiameter,
                            Encoder.TalonFXIntegrated);
        }
        return sum / motors.length;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Absolute Angle", elevatorEncoder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position1 ", motors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position2 ", motors[1].getSelectedSensorPosition());
    }

}