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
    
    // Kraken motor controlling the turret
    private final TalonFX turretMotor;

    // PID controller for the angle of turret
    private final PIDController anglePidController;

    // Absolute encoder used to get the current angle of the turret
    private final CANcoder absEncoder;

    // If the absolute encoder is reversed
    private final boolean absEncoderReversed;

    public Turret(int turretID, int encoderID, boolean absEncoderReversed) {
        // Initializations
        this.turretMotor = new TalonFX(turretID);

        this.anglePidController = new PIDController(
            TurretConstants.turretP,
            TurretConstants.turretI,
            TurretConstants.turretD
        );

        this.anglePidController.setSetpoint(0);

        this.absEncoder = new CANcoder(encoderID);
        this.absEncoderReversed = absEncoderReversed;

        // Setup preferences (only temporary since it will be deleted after first initialization)
        if (Preferences.getDouble("TurretZero", 200) == 200) {
            Preferences.initDouble("TurretZero", 0);
        }
    }

    /*  
     * Gets the angle of the absolute encoder, in rotations,
     * and then converts to degrees and reverses if necessary 
     */
    public double getAngle() {
        double rotations = this.absEncoder.getAbsolutePosition().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(rotations);

        if (absEncoderReversed) {
            angle.unaryMinus();
        }

        return angle.getDegrees();
    }
    
    /*
     * Does some basic math to reset the turret's zero position
     */
    public void resetEncoder() {
        double offset = Rotation2d.fromDegrees(Preferences.getDouble("TurretZero", 0)).plus(Rotation2d.fromDegrees(getAngle())).getRotations();
        Preferences.setDouble("TurretZero", Rotation2d.fromRotations(offset).getDegrees());
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        absEncoder.getConfigurator().apply(config);
    }


    /*
     * Called every 20 ms (command schedule loop)
     */
    @Override
    public void periodic() {
        // Gets the pid controller's output
        double currentAngle = this.getAngle();
        double calculatedOutput = this.anglePidController.calculate(currentAngle);

        // Makes sure the turret doesn't over extend
        boolean overrun = (
            (calculatedOutput < 0 && currentAngle < TurretConstants.TURRETMINANGLE) ||
            (calculatedOutput > 0 && currentAngle > TurretConstants.TURRETMAXANGLE)
        );

        // Sets the correct voltage for the motor
        double finalOutput = overrun ? 0 : calculatedOutput;
        this.turretMotor.setVoltage(finalOutput);
    }

    /*
     * Adds values that can be displayed on Elastic dashboard
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Turret Angle",
            this::getAngle,
            null
        );
    }
}
