package adaptive_pure_pursuit.specific;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import geometry.InterpolatingDouble;
import geometry.InterpolatingTreeMap;
import geometry.RigidTransform;
import geometry.Rotation;
import geometry.Twist;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout the match. A coordinate frame is simply a
 * point and direction in space that defines an (x,y) coordinate system. Transforms (or poses) keep track of the spatial
 * relationship between different frames.
 *
 * Robot frames of interest (from parent to child):
 *
 * 1. Field frame: origin is where the robot is turned on
 *
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing forwards
 *
 *
 * As a kinematic chain with 2 frames, there is 1 transform of interest:
 *
 * Field-to-vehicle: This is tracked over time by integrating encoder and gyro measurements. It will inevitably
 * drift, but is usually accurate over short time periods.
 */

public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    private static final int kObservationBufferSize = 100;

    // FPGATimestamp -> RigidTransform or Rotation
    private InterpolatingTreeMap<InterpolatingDouble, RigidTransform> field_to_vehicle_;
    private Twist vehicle_velocity_predicted_;
    private Twist vehicle_velocity_measured_;
    private double distance_driven_;

    private RobotState() {
        reset(0, new RigidTransform());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, RigidTransform initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist.identity();
        vehicle_velocity_measured_ = Twist.identity();
        distance_driven_ = 0.0;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized RigidTransform getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized RigidTransform getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(RigidTransform.exp(vehicle_velocity_predicted_.scale(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, RigidTransform observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist measured_velocity,
            Twist predicted_velocity) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), measured_velocity));
        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation current_gyro_angle) {
        final RigidTransform last_measurement = getLatestFieldToVehicle().getValue();
        final Twist delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
        distance_driven_ += delta.dx;
        return delta;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized Twist getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public void outputToSmartDashboard() {
        RigidTransform odometry = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().x());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().y());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        SmartDashboard.putNumber("robot velocity", vehicle_velocity_measured_.dx);
    }
}
