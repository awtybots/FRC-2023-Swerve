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

    public double elevatorTargetHeight = Constants.ElevatorConstants.initialHeight;
    
    
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
        return elevatorEncoder.getSelectedSensorPosition()*5;
    }

    private void configElevatorEncoder(){        
        elevatorEncoder.configFactoryDefault();
        elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        

        // elevatorEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();

        mRightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        for (WPI_TalonFX motor : motors) {
            //motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            //motor.setSelectedSensorPosition(0.0);

            motor.setNeutralMode(NeutralMode.Brake);


            motor.configOpenloopRamp(kRamp);
            motor.configClosedloopRamp(kRamp);
            motor.configPeakOutputForward(kMaxPercentOutput);
            motor.configPeakOutputReverse(-kMaxPercentOutput);
            motor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
        }

        resetToAbsolute();
    }

    public void setHeight(int value){
        elevatorTargetHeight = value;
    }

    public void drive(double pct) {
        if(elevatorTargetHeight > Constants.ElevatorConstants.maximumHeight && pct > 0) return;
        if(elevatorTargetHeight < Constants.ElevatorConstants.minimumHeight && pct < 0) return;
        motors[0].set(ControlMode.Position, elevatorTargetHeight);
        motors[1].set(ControlMode.Position, elevatorTargetHeight);
        elevatorTargetHeight += pct*1000;
    }

    public void stop() {
        for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Absolute Angle", elevatorEncoder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position1 ", motors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position2 ", motors[1].getSelectedSensorPosition());


        //3868 -> 5828  =  1960
        //15067 -> 24873   =  9806 

        for (WPI_TalonFX motor : motors){
            motor.config_kP(0, Constants.ElevatorConstants.kP);
            motor.config_kI(0, Constants.ElevatorConstants.kI);
            motor.config_kD(0, Constants.ElevatorConstants.kD);
            motor.config_kF(0,Constants.ElevatorConstants.kF);
        }



    }

}