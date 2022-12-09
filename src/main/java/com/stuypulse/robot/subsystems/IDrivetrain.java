package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IDrivetrain extends SubsystemBase {
    public abstract void setTargetSpeeds(double leftVelocity, double rightVelocity);
    public abstract DifferentialDriveWheelSpeeds getSpeeds();

    public abstract double getLeftDistance();
    public abstract double getRightDistance();

    public abstract Rotation2d getRotation2d();

    public abstract Pose2d getPose();
    // public abstract void setPose
}
