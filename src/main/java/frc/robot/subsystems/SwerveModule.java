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
    private final PIDController driveVelocityPidController;
    private final SlewRateLimiter driveVelocityLimiter = new SlewRateLimiter(3);;

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

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Rotation2d.fromDegrees(Preferences.getDouble("Module" + this.swerveID + "Zero", 0)).getRotations();
        this.absoluteEncoder.getConfigurator().apply(config);

        this.driveMotor = new TalonFX(driveMotorId);
        this.turningMotor = new TalonFX(turningMotorId);

        this.driveMotor.getConfigurator().apply(getDriveConfig(driveMotorReversed));
        this.turningMotor.getConfigurator().apply(getTurningConfig());

        this.turningPidController = new PIDController(
            Preferences.getDouble("Module" + this.swerveID + "P", SwerveConstants.SWERVETURNINGP),
            0,
            Preferences.getDouble("Module" + this.swerveID + "D", SwerveConstants.SWERVETURNINGD)
        );
        this.turningPidController.enableContinuousInput(-180, 180);
        this.turningPidController.setTolerance(0.2);
        
        this.driveVelocityPidController = new PIDController(
            Preferences.getDouble("Module" + this.swerveID + "VelocityP", 1),
            0,
            0
        );
        this.driveVelocityPidController.setTolerance(0.2);

        this.driveRotationToMeter = (gearRatio == 1) ? SwerveConstants.ROTATIONSTOMETERSR1 : (gearRatio == 2) ? SwerveConstants.ROTATIONSTOMETERSR2 : SwerveConstants.ROTATIONSTOMETERSR3;
        this.driveMaxSpeed = (gearRatio == 1) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR1 : (gearRatio == 2) ? SwerveConstants.PHYSICALMAXSPEEDMPERSECR2 : SwerveConstants.PHYSICALMAXSPEEDMPERSECR3;

        this.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(getAbsoluteEncoderDeg())));
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

    public void resetEncoders() {
        // System.out.println("1 " + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
        this.driveMotor.setPosition(0);
        double offset = Rotation2d.fromDegrees(Preferences.getDouble("Module" + this.swerveID + "Zero", 0)).plus(Rotation2d.fromDegrees(getAbsoluteEncoderDeg())).getRotations();
        Preferences.setDouble("Module" + this.swerveID + "Zero", Rotation2d.fromRotations(offset).getDegrees());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        this.absoluteEncoder.getConfigurator().apply(config);
        // System.out.println("2" + this.swerveID + " " + getAbsoluteEncoderDeg() + "\n");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void runVolts(double volts) {
        this.driveMotor.setVoltage(volts);
    }

    public void periodic() {
        double limitedSpeed = driveVelocityLimiter.calculate(this.state.speedMetersPerSecond);
        this.driveVelocityPidController.setSetpoint(limitedSpeed);
        //System.out.println(this.swerveID + " Desired Angle: " + this.state.angle.getDegrees());
        this.driveMotor.set(this.driveVelocityPidController.calculate(this.getDriveVelocity()) / this.driveMaxSpeed);
        this.turningMotor.setVoltage(this.turningPidController.calculate(getAbsoluteEncoderDeg()));
        SmartDashboard.putNumber(this.swerveID + " Module Angle: ", getAbsoluteEncoderDeg());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            desiredState.speedMetersPerSecond = 0;
        }
        desiredState.optimize(Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
        this.state = desiredState;
        turningPidController.setSetpoint(this.state.angle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition position = new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
        return position;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public int getSwerveID() {
        return this.swerveID;
    }

    public PIDController getTurningController() {
        return this.turningPidController;
    }

    public PIDController getDriveController() {
        return this.driveVelocityPidController;
    }

    public void setTurningControllerP(double p) {
        Preferences.setDouble("Module_" + swerveID + "_P", p);
        this.turningPidController.setP(p);
    }

    public void setTurningControllerD(double d) {
        Preferences.setDouble("Module_" + swerveID + "_D", d);
        this.turningPidController.setD(d);
    }

    public void setDriveControllerP(double p) {
        Preferences.setDouble("Module_" + this.swerveID + "_P", p);
        this.driveVelocityPidController.setP(p);
    }
}