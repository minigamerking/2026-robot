package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    private int[] aprilTagFilter = {2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27};

    public Vision() {
        applyAprilTagFilter(aprilTagFilter);
    }

    private final String[] limelights = {
        "limelight-shooter" //10.17.11.11
    };

    public Pose2d estimatePose() {
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelights[0]);

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            return llMeasurement.pose;
        }

        return null;
    }

    public void applyAprilTagFilter(int[] filter) {
        this.aprilTagFilter = filter;

        LimelightHelpers.SetFiducialIDFiltersOverride(limelights[0], filter);
    }

    public double getDistance() {
        double targetOffsetAngle = LimelightHelpers.getTY(limelights[0]);

        double llAngleOffsetDegrees = 20;
        double llLensHeightInches = 7.46114106;

        double targetHeightInches = 44.25;

        double angleToTargetDegrees = llAngleOffsetDegrees + targetOffsetAngle;
        double angleToTargetRadians = Units.degreesToRadians(angleToTargetDegrees);

        double llToTargetInches = (targetHeightInches - llLensHeightInches) / Math.tan(angleToTargetRadians);

        return llToTargetInches;
    }

    public List<VisionMeasurement> getVisionMeasurements() {
        List<VisionMeasurement> measurements = new ArrayList<>();

        for (String ll : limelights) {
            var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

            if (estimate != null && estimate.tagCount >= 2) {
                measurements.add(
                    new VisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds
                    )
                );
            }
        }

        return measurements;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Distance to Hub", 
            this::getDistance, 
            null
        );
    }

    public record VisionMeasurement(Pose2d pose, double timestampSeconds) {}
}