package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SAT.SAT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Trigger podium_trigger = new Trigger(() -> goToPodiumInput());
    private final Trigger sub_trigger = new Trigger(() -> goToSubInput());
    private final Trigger amp_trigger = new Trigger(() -> goToAmpInput());
    private final Trigger trap_trigger = new Trigger(() -> goToTrapInput());

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public SAT m_SAT = new SAT();

     /* AutoChooser */
     private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
        
        //Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
         
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        podium_trigger.onTrue(new InstantCommand(() -> m_SAT.goToPodiumPosition()));
        sub_trigger.onTrue(new InstantCommand(() -> m_SAT.goToSubPosition()));
        amp_trigger.onTrue(new InstantCommand(() -> m_SAT.goToAmpPosition()));
        trap_trigger.onTrue(new InstantCommand(()-> m_SAT.goToTrapPosition()));
    }

    public boolean goToPodiumInput() {
        return (operator.getRawButton(5) && (operator.getRawAxis(1) > 0.85) && (Math.abs(operator.getRawAxis(0)) < 0.02));
    }

    public boolean goToSubInput() {
        return (operator.getRawButton(5) && (operator.getRawAxis(1) < -0.85) && (Math.abs(operator.getRawAxis(0)) < 0.02));
    }

    public boolean goToAmpInput() {
        return (operator.getRawButton(5) && (operator.getRawAxis(0) < -0.85) && (Math.abs(operator.getRawAxis(1)) < 0.02));
    }

    public boolean goToTrapInput() {
        return (operator.getRawButton(5) && (operator.getRawAxis(0) > 0.85) && (Math.abs(operator.getRawAxis(1)) < 0.02));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
