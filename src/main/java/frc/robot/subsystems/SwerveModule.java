package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;
    private final SlewRateLimiter driveVelocityLimiter = new SlewRateLimiter(15, -25, 0);

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private SwerveModuleState state;

    private final int swerveID;

    private final double driveRotationToMeter;
    private final double driveMaxSpeed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
            int absoluteEncoderId, boolean absoluteEncoderReversed, int swerveID, int gearRatio) {

        this.swerveID = swerveID;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = new CANcoder(absoluteEncoderId);

        Preferences.initDouble("Module" + this.swerveID + "VelocityI", 1);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Rotation2d.fromRadians(Preferences.getDouble("Module" + this.swerveID + "Zero", 0)).getRotations();
        this.absoluteEncoder.getConfigurator().apply(config);

        this.driveMotor = new TalonFX(driveMotorId);
        this.turningMotor = new TalonFX(turningMotorId);

        this.driveMotor.getConfigurator().apply(getDriveConfig(driveMotorReversed));
        this.turningMotor.getConfigurator().apply(getTurningConfig());

        this.turningPidController = new PIDController(
            Preferences.getDouble("Module" + this.swerveID + "TurningP", SwerveConstants.SWERVETURNINGP),
            0,
            Preferences.getDouble("Module" + this.swerveID + "TurningD", SwerveConstants.SWERVETURNINGD)
        );
        this.turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        this.turningPidController.setTolerance(0.1);

        this.driveRotationToMeter = (gearRatio == 1) ? SwerveConstants.ROTATIONSTOMETERSR1 : (gearRatio == 2) ? SwerveConstants.ROTATIONSTOMETERSR2 : SwerveConstants.ROTATIONSTOMETERSR3;
        this.driveMaxSpeed = (gearRatio == 1) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR1 : (gearRatio == 2) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR2 : SwerveConstants.PHYSICALMAXSPEEDMPERSECR3;

        this.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(getAbsoluteEncoderRad())));
    }

    private TalonFXConfiguration getDriveConfig(boolean reversed) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput
            .Inverted = reversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        return config;
    }

    private TalonFXConfiguration getTurningConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        return config;
    }

    public double getDrivePosition() {
        return this.driveMotor.getPosition().getValueAsDouble() * this.driveRotationToMeter;
    }

    public double getDriveVelocity() {
        return this.driveMotor.getVelocity().getValueAsDouble() * this.driveRotationToMeter;
    }

    public double getAbsoluteEncoderDeg() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absoluteEncoderReversed) {
            angle = angle.unaryMinus();
        }

        return angle.getDegrees();
    }

    public double getAbsoluteEncoderRad() {
        double rotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absoluteEncoderReversed) {
            angle = angle.unaryMinus();
        }

        return angle.getRadians();
    }

    public void resetEncoders() {
        // System.out.println("1 " + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
        this.driveMotor.setPosition(0);
        double offset = Rotation2d.fromRadians(Preferences.getDouble("Module" + this.swerveID + "Zero", 0)).plus(Rotation2d.fromRadians(getAbsoluteEncoderRad())).getRotations();
        Preferences.setDouble("Module" + this.swerveID + "Zero", Rotation2d.fromRotations(offset).getRadians());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        this.absoluteEncoder.getConfigurator().apply(config);
        // System.out.println("2" + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

    public void runVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public void periodic() {
        double limitedSpeed = driveVelocityLimiter.calculate(this.state.speedMetersPerSecond);
        SmartDashboard.putNumber(this.swerveID + " Slewed Velocity", limitedSpeed);
        //System.out.println(this.swerveID + " Desired Angle: " + this.state.angle.getDegrees());
        this.driveMotor.set(limitedSpeed / this.driveMaxSpeed);
        this.turningMotor.setVoltage(this.turningPidController.calculate(getAbsoluteEncoderRad()));
        SmartDashboard.putNumber(this.swerveID + " Velocity", this.getDriveVelocity());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.05) {
            desiredState.speedMetersPerSecond = 0;
        }
        desiredState.optimize(getState().angle);
        this.state = desiredState;
        SmartDashboard.putNumber(this.swerveID+ " Velocity Setpoint", this.state.speedMetersPerSecond);
        turningPidController.setSetpoint(this.state.angle.getRadians());
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getAbsoluteEncoderRad()));
        return position;
    }

    public void stop() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    public int getSwerveID() {
        return this.swerveID;
    }

    public PIDController getTurningController() {
        return this.turningPidController;
    }

    public void setTurningControllerP(double p) {
        Preferences.setDouble("Module" + swerveID + "TurningP", p);
        this.turningPidController.setP(p);
    }

    public void setTurningControllerD(double d) {
        Preferences.setDouble("Module" + swerveID + "TurningD", d);
        this.turningPidController.setD(d);
    }
}