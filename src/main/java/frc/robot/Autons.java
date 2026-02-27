package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Highway;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class Autons {
    private final AutoFactory autoFactory;
    private final Shooter shooter;
    private final Highway highway;

    public Autons(SwerveSubsystem swerve, Shooter shooter, Highway highway) {
        this.autoFactory = new AutoFactory(
            swerve::getPose, 
            swerve::resetOdometry, 
            swerve::followTrajectory, 
            true, 
            swerve
        );

        this.shooter = shooter;
        this.highway = highway;
    }

    public Command lastResortAuton() {
        return Commands.sequence(
            autoFactory.resetOdometry("SimpleLeave"),
            autoFactory.trajectoryCmd("SimpleLeave")
        ).withName("Last Resort Auton");
    }

    public AutoRoutine fireAndAscentAuton() {
        AutoRoutine autoRoutine = autoFactory.newRoutine("fireAndAscent");

        AutoTrajectory shootAndClimb = autoRoutine.trajectory("ShootAndClimb");

        autoRoutine.active().onTrue(
            Commands.sequence(
                shootAndClimb.resetOdometry(),
                shooter.commands.shootVelocity(),
                shootAndClimb.cmd()
                //climber.commands.climb()
            )
        );

        return autoRoutine;
    }
}
