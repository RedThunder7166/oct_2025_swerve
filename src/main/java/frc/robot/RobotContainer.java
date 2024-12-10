// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final CommandXboxController driver_joystick = new CommandXboxController(0);
    private final CANdleSubsystem candle = new CANdleSubsystem();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver_joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver_joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver_joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver_joystick.getLeftY(), -driver_joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver_joystick.back().and(driver_joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver_joystick.back().and(driver_joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver_joystick.start().and(driver_joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver_joystick.start().and(driver_joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver_joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        if (Utils.isSimulation()) {
            drivetrain.resetPose(new Pose2d());
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public void configureNamedCommands() {
        NamedCommands.registerCommand("IncrementLED", new InstantCommand(candle::incrementAnimation, candle));
    }

    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        configureNamedCommands();

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
        // return new PathPlannerAuto("TheAuto");
    }
}
