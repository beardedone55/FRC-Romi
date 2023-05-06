// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveTrain.kDriveVelocity;
import static frc.robot.Constants.DriveTrain.kMaxAccelerationMetersPerSecondSquared;
import static frc.robot.Constants.DriveTrain.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.DriveTrain.kRamseteB;
import static frc.robot.Constants.DriveTrain.kRamseteZeta;
import static frc.robot.Constants.DriveTrain.kaVoltSecondsSquaredPerMeter;
import static frc.robot.Constants.DriveTrain.kinematics;
import static frc.robot.Constants.DriveTrain.ksVolts;
import static frc.robot.Constants.DriveTrain.kvVoltSecondsPerMeter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RomiDrivetrain;


public class TrajectoryTest extends RamseteCommand {
    /**
     * Creates a new TrajectoryTest.
     *
     * @param subsystem The subsystem used by this command.
     */

    private static final TrajectoryConfig trajectoryConfig = getTrajectoryConfig();
    private static final Trajectory trajectory = getTrajectory();

    public TrajectoryTest(RomiDrivetrain drivebase) {
        super(
            trajectory,
            drivebase::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
            kinematics,
            drivebase::getWheelSpeeds,
            new PIDController(kDriveVelocity, 0, 0),
            new PIDController(kDriveVelocity, 0, 0),
            drivebase::tankDriveVolts,
            drivebase
        );

        drivebase.resetOdometry(trajectory.getInitialPose());
    }

    private static DifferentialDriveVoltageConstraint getVoltageConstraint() {
        return new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                ksVolts, 
                kvVoltSecondsPerMeter, 
                kaVoltSecondsSquaredPerMeter),
            kinematics,
            8);
    }

    private static TrajectoryConfig getTrajectoryConfig() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond, 
                kMaxAccelerationMetersPerSecondSquared);
        trajectoryConfig
             .setKinematics(kinematics);
             //.addConstraint(getVoltageConstraint());
        return trajectoryConfig;
    }

    private static Trajectory getTrajectory() {
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(1,1));
        //waypoints.add(new Translation2d(2,-1));

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0,new Rotation2d(0)), 
            waypoints,
            new Pose2d(3, 0, new Rotation2d(0)), 
            trajectoryConfig);
    }
}
