package frc.robot;

import javax.print.DocFlavor.STRING;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final Joystick driver = new Joystick(0);
    // private final Joystick operator = new Joystick(2);
    private final Buttons m_stupidButtons = new Buttons();
    private final XboxController m_Operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton clawToggle = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton armUp = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton armDown = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton armOffset = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton wristLeft = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton wristRight = new JoystickButton(driver, XboxController.Button.kY.value);

    private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

    /* operator Buttons */
    private final JoystickButton loadingButton = new JoystickButton(m_Operator, XboxController.Button.kX.value);
    private final JoystickButton lowButton = new JoystickButton(m_Operator, XboxController.Button.kA.value);
    private final JoystickButton mediumButton = new JoystickButton(m_Operator, XboxController.Button.kB.value);
    private final JoystickButton highButton = new JoystickButton(m_Operator, XboxController.Button.kY.value);
    private final JoystickButton rightBumper = new JoystickButton(m_Operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftBumper = new JoystickButton(m_Operator, XboxController.Button.kLeftBumper.value);

 //   private final JoystickButton nodeOne = new JoystickButton(operator, 1);
   // private final JoystickButton nodeTwo = new JoystickButton(operator, 2);
    //   private final JoystickButton wristUpButton = new JoystickButton(operator, 1);
    //   private final JoystickButton wristDownButton = new JoystickButton(operator, 2);
    //   private final JoystickButton armUpButton = new JoystickButton(operator, 4);
    //   private final JoystickButton armDownButton = new JoystickButton(operator, 5);
    //   private final JoystickButton gripperCloseCone = new JoystickButton(operator, 0);
    //   private final JoystickButton gripperCloseCube = new JoystickButton(operator, 0);
    //   private final JoystickButton gripperOpen = new JoystickButton(operator, 0);
    //   private final JoystickButton EMERGENCYSTOP = new JoystickButton(operator, 0);




    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    // private final PnuematicSubsystem s_PneumaticsHub = new PnuematicSubsystem();
    private final theCLAAAWWW s_Claaawww = new theCLAAAWWW();
    private final GripperSubsystem s_GripperSubsystem = new GripperSubsystem();
    private final Wrist s_wrist = Wrist.getInstance();

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
                        () -> robotCentric.getAsBoolean()));

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

        PathPlannerTrajectory thingTraj = PathPlanner.loadPath("thing", new PathConstraints(3, 5));

        autoChooser.setDefaultOption("Thing", thingTraj); //TODO: Change default auto
        autoChooser.addOption("Thing", thingTraj);

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

        // when up button is pressed, start moving claw up
        // when down button is pressed, start moving the claw down
        // armUp.onTrue(new InstantCommand(() -> s_Claaawww.armUp()));
        // armDown.onTrue(new InstantCommand(() -> s_Claaawww.armDown()));
        // wristLeft.onTrue(new InstantCommand(() -> s_Claaawww.wristLeft()));
        // wristRight.onTrue(new InstantCommand(() -> s_Claaawww.wristRight()));

        // when up or down button is unpressed, stop moving the claw
        // when stop button is pressed, stop moving the claw
        // armUp.onFalse(new InstantCommand(() -> s_Claaawww.armStop()));
        // armDown.onFalse(new InstantCommand(() -> s_Claaawww.armStop()));
        // armOffset.onTrue(new InstantCommand(() -> s_Claaawww.setArmOffsets()));
        // wristLeft.onFalse(new InstantCommand(() -> s_Claaawww.wristStop()));
        // wristRight.onFalse(new InstantCommand(() -> s_Claaawww.wristStop()));

        /* operator Buttons */

        // nodeOne.onTrue(new InstantCommand(() ->
        // s_Claaawww.setClawstate(ClawState.LOADING)));
        // nodeOne.onTrue(new InstantCommand(() -> SmartDashboard.putBoolean("nodeOne",
        // true)));
        // nodeOne.onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("nodeOne",
        // false)));

        // wristUpButton.onTrue(new InstantCommand(() -> s_wrist.driveWrist(0.1)));
        // wristUpButton.onFalse(new InstantCommand(() -> s_wrist.driveWrist(0)));

        // wristDownButton.onTrue(new InstantCommand(() -> s_wrist.driveWrist(-0.1)));
        // wristDownButton.onFalse(new InstantCommand(() -> s_wrist.driveWrist(0)));

        // armUpButton.onTrue(new InstantCommand(() -> s_Claaawww.drive(0.1)));
        // armUpButton.onFalse(new InstantCommand(() -> s_Claaawww.drive(0)));

        // armDownButton.onTrue(new InstantCommand(() -> s_Claaawww.drive(-0.1)));
        // armDownButton.onFalse(new InstantCommand(() -> s_Claaawww.drive(0)));

        

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
        return new autoCommandGroup(s_Swerve, s_Claaawww);
    }
}
