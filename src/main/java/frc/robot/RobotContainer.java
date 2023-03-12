package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.print.DocFlavor.STRING;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.theCLAAAWWW.ClawState;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final Joystick driver = new Joystick(0);
    private final XboxController driver = new XboxController(0);
    // private final Joystick operator = new Joystick(2);
    private final Buttons m_stupidButtons = new Buttons();
    private final XboxController m_Operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton fastTurnLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton fastTurnRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton armUp = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton resetOdometry = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton armOffset = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton wristLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton wristRight = new JoystickButton(driver, XboxController.Button.kY.value);

    // private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();
    private final SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

    /* operator Buttons */
    private final JoystickButton loadingButton = new JoystickButton(m_Operator, XboxController.Button.kX.value);
    private final JoystickButton lowButton = new JoystickButton(m_Operator, XboxController.Button.kA.value);
    private final JoystickButton mediumButton = new JoystickButton(m_Operator, XboxController.Button.kB.value);
    private final JoystickButton highButton = new JoystickButton(m_Operator, XboxController.Button.kY.value);
    private final JoystickButton rightBumper = new JoystickButton(m_Operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftBumper = new JoystickButton(m_Operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton back = new JoystickButton(m_Operator, XboxController.Button.kBack.value);


    /* Subsystems */
    private static Swerve s_Swerve = new Swerve();
    // private final PnuematicSubsystem s_PneumaticsHub = new PnuematicSubsystem();
    private final theCLAAAWWW s_Claaawww = new theCLAAAWWW();
    private final GripperSubsystem s_GripperSubsystem = new GripperSubsystem();
    // private final Wrist s_wrist = Wrist.getInstance();
    private final PathPlaaanner s_PathPlanner = new PathPlaaanner(s_Swerve);
    private final LimelightSubsystem l_Limelight = new LimelightSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Shuffleboard.getTab("Autonomous").add(autoChooser);

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> Math.pow(-driver.getRawAxis(translationAxis), 3),
                        () -> Math.pow(-driver.getRawAxis(strafeAxis), 3),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> false));
                        
        s_GripperSubsystem.setDefaultCommand(
            new RunCommand(
                () -> s_GripperSubsystem
                                .driveGripper(m_Operator.getRightTriggerAxis() - m_Operator.getLeftTriggerAxis()),
                                s_GripperSubsystem));

        // s_wrist.setDefaultCommand(
            //         new RunCommand(
        //                 () -> s_wrist.driveWrist(m_Operator.getLeftY()),
        //                 s_wrist)
        // );

        s_Claaawww.setDefaultCommand(new RunCommand(() -> s_Claaawww.drive(-m_Operator.getRightY(), m_Operator.getLeftY()), s_Claaawww));

        // autoChooser.setDefaultOption("Thing", thingTraj); //TODO: Change default auto
        // autoChooser.addOption("Thing", thingTraj);

        //autoChooser.setDefaultOption("Place Drive", m_placeDriveAuto);
        autoChooser.addOption("PlaceAutoLeft", new PlaceAutoLeft(s_Swerve, s_Claaawww, s_GripperSubsystem));
        autoChooser.addOption("RightChargingStation", new RightChargingStation(s_Swerve));
        autoChooser.addOption("DropCenterChargeAuto", new DropCenterChargeAuto(s_Swerve, s_Claaawww, s_GripperSubsystem));
        autoChooser.addOption("PlaceAutoRight", new PlaceAutoRight(s_Swerve, s_Claaawww, s_GripperSubsystem));
        autoChooser.addOption("LeftHighAuto", new LeftHighAuto(s_Swerve, s_Claaawww, s_GripperSubsystem));
        autoChooser.addOption("RightHighAuto", new RightHighAuto(s_Swerve, s_Claaawww, s_GripperSubsystem));
        autoChooser.addOption("Test Auto", new TestAuto(s_Swerve, s_Claaawww ,s_GripperSubsystem));
        autoChooser.addOption("1 meter", s_PathPlanner.newFullAuto(PathPlanner.loadPath("1 meter", 3, 5)));
        autoChooser.addOption("TwoPieceAuto", new TwoPieceAuto(s_Swerve, s_Claaawww, s_GripperSubsystem));
        // Configure the button bindings

        configureButtonBindings();
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private static boolean[] buttonStates = {
        false,
        false,
        false,
        false
    };

    public void stopMotors(){
        s_Claaawww.drive(0, 0);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetOdometry.onTrue(new InstantCommand(()-> s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()))));
      //  back.onTrue(s_GripperSubsystem.toggleLimitedControl());

        fastTurnLeft.whileTrue(new TeleopSwerve(
            s_Swerve,
            () -> Math.pow(-driver.getRawAxis(translationAxis), 3),
            () -> Math.pow(-driver.getRawAxis(strafeAxis), 3),
            () -> -2,
            () -> false));
        fastTurnRight.whileTrue(new TeleopSwerve(
            s_Swerve,
            () -> Math.pow(-driver.getRawAxis(translationAxis), 3),
            () -> Math.pow(-driver.getRawAxis(strafeAxis), 3),
            () -> 2,
            () -> false));
        
        // lowButton.onTrue(new ClawCommand(s_Claaawww, ClawState.LOW));
        // highButton.onTrue(new ClawCommand(s_Claaawww, ClawState.HIGH));
        // loadingButton.onTrue(new ClawCommand(s_Claaawww, ClawState.LOADING));
        // mediumButton.onTrue(new ClawCommand(s_Claaawww, ClawState.MEDIUM));
        // leftBumper.onTrue(new ClawCommand(s_Claaawww, ClawState.TRANSPORT));

        // //s_Claaawww.setState(ClawState.LOADING);

        // rightBumper.whileTrue(new RunCommand(() -> s_Claaawww.drive(-m_Operator.getRightY(), m_Operator.getLeftY()), s_Claaawww));
        // rightBumper.onFalse(new RunCommand(() -> s_Claaawww.drive(0, 0), s_Claaawww));
        //Set arm speed to FF*cos(getArmAngle)
        //e.g. .05*sin(0) = 0
        //e.g. .05sin(90) = .05

        // clawToggle.onTrue(new InstantCommand(() ->
        // s_PneumaticsHub.toggleClawSolenoid()));

    }

    /**
     * Use this to pass the autonomous c
     * command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An ExampleCommand will run in autonomous
        // return s_Swerve.followTrajectoryCommand(autoChooser.getSelected(), true);
        // return new autoCommandGroup(s_Swerve, s_Claaawww);
        return autoChooser.getSelected();
    }
}
