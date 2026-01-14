package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    
    private final TalonFX turretMotor;

    private final PIDController anglePidController;

    private final CANcoder absEncoder;

    private final boolean absEncoderReversed;

    public Turret(int turretID, int encoderID, boolean absEncoderReversed) {
        this.turretMotor = new TalonFX(turretID);

        this.anglePidController = new PIDController(
            TurretConstants.turretP,
            TurretConstants.turretI,
            TurretConstants.turretD
        );

        this.anglePidController.setSetpoint(0);

        this.absEncoder = new CANcoder(encoderID);
        this.absEncoderReversed = absEncoderReversed;

        if (Preferences.getDouble("TurretZero", 200) == 200) {
            Preferences.initDouble("TurretZero", 0);
        }
    }

    public double getAngle() {
        double rotations = this.absEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absEncoderReversed) {
            angle.unaryMinus();
        }

        return angle.getDegrees();
    }
    
    public void resetEncoder() {
        double offset = Rotation2d.fromDegrees(Preferences.getDouble("TurretZero", 0)).plus(Rotation2d.fromDegrees(getAngle())).getRotations();
        Preferences.setDouble("TurretZero", Rotation2d.fromRotations(offset).getDegrees());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absEncoder.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        double currentAngle = this.getAngle();
        double calculatedOutput = this.anglePidController.calculate(currentAngle);

        boolean overrun = (
            (calculatedOutput < 0 && currentAngle < TurretConstants.TURRETMINANGLE) ||
            (calculatedOutput > 0 && currentAngle > TurretConstants.TURRETMAXANGLE)
        );

        double finalOutput = overrun ? 0 : calculatedOutput;

        this.turretMotor.setVoltage(finalOutput);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Turret Angle",
            this::getAngle,
            null
        );
    }
}
