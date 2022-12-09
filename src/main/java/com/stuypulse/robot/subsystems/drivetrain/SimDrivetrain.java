package com.stuypulse.robot.subsystems.drivetrain;

import java.util.List;
import java.util.function.BiConsumer;

import com.stuypulse.robot.subsystems.IDrivetrain;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimDrivetrain extends IDrivetrain {

    // Simulation
    private DifferentialDrivetrainSim truth; // represents real pose of the robot
    private DifferentialDrivetrainSim simulation; // represents sensor readings of the robot (noise)
    
    // Control
    private Controller leftController;
    private Controller rightController;

    private double leftTargetSpeed;
    private double rightTargetSpeed;

    // Odometry
    private DifferentialDrivePoseEstimator odometry;
    private Field2d field2d;

    public SimDrivetrain() {
        // Simulation
        truth = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                1.7, 0.45, 
                6.33, 1.35
            ),
            DCMotor.getNEO(3),
            1800.0 / 408.0,
            Units.inchesToMeters(30.0),
            Units.inchesToMeters(4.0),

            null
        );

        simulation = new DifferentialDrivetrainSim(
            LinearSystemId.identifyDrivetrainSystem(
                1.7, 0.45, 
                6.33, 1.35
            ),
            DCMotor.getNEO(3),
            1800.0 / 408.0,
            Units.inchesToMeters(30.0),
            Units.inchesToMeters(4.0),

            new MatBuilder<N7, N1>(Nat.N7(), Nat.N1())
                .fill(0.01, 0.01, 0.0001, 0.05, 0.05, 0.005, 0.005)
        );

        // Control
        leftController = new PIDController(1.0, 0.0, 0.0)
            .add(new Feedforward.Drivetrain(0.0, 1.7, 0.45).velocity());
        
        rightController = new PIDController(1.0, 0.0, 0.0)
            .add(new Feedforward.Drivetrain(0.0, 1.7, 0.45).velocity());

        leftTargetSpeed = 0.0;
        rightTargetSpeed = 0.0;

        // Odometry        
        odometry = new DifferentialDrivePoseEstimator(
            getRotation2d(),
            simulation.getLeftPositionMeters(),
            simulation.getRightPositionMeters(),
            new Pose2d(),
            new MatBuilder<N5, N1>(Nat.N5(), Nat.N1())
                .fill(0.01, 0.01, 0.0001, 0.005, 0.005), // standard deviation of the estimated state  
            new MatBuilder<N3, N1>(Nat.N3(), Nat.N1())
                .fill(0.005, 0.005, 0.0001), // standard devations of the local measurements 
            new MatBuilder<N3, N1>(Nat.N3(), Nat.N1())
                .fill(0, 0, 0) // how much to trust global estimate
        );

        field2d = new Field2d();
        SmartDashboard.putData(field2d);
    }

    @Override 
    public Rotation2d getRotation2d() {
        return simulation.getHeading();
    }

    @Override
    public void setTargetSpeeds(double leftVelocity, double rightVelocity) {
        leftTargetSpeed = leftVelocity;
        rightTargetSpeed = rightVelocity;
    }

    @Override
    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            simulation.getLeftVelocityMetersPerSecond(),
            simulation.getRightVelocityMetersPerSecond()
        );
    }

    @Override
    public double getLeftDistance() {
        return simulation.getLeftPositionMeters();
    }

    @Override
    public double getRightDistance() {
        return simulation.getRightPositionMeters();
    }

    @Override
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        List<DifferentialDrivetrainSim> models = List.of(truth, simulation);
        for (var model : models) {
            model.setInputs(
                leftController.update(leftTargetSpeed, model.getLeftVelocityMetersPerSecond()),
                rightController.update(rightTargetSpeed, model.getRightVelocityMetersPerSecond())
            );    

            model.update(0.02);
        }
        
        field2d.getObject("real pose").setPose(truth.getPose());

        odometry.update(getRotation2d(), getSpeeds(), getLeftDistance(), getRightDistance());
        odometry.addVisionMeasurement(truth.getPose(), Timer.getFPGATimestamp());

        field2d.getObject("estimated pose").setPose(odometry.getEstimatedPosition());
        field2d.getObject("sensor pose").setPose(simulation.getPose());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(simulation.getCurrentDrawAmps())
        );
    }
    
}
