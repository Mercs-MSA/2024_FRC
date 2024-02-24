package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandShootNote;
import frc.robot.commands.CommandStopShooter;
import frc.robot.commands.CommandPivotHandoffPosition;
import frc.robot.commands.CommandMovePivotToPosition;
import frc.robot.commands.CommandChangeScoringMode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.IntakeSubcommands.*;
import frc.robot.commands.IndexSubcommands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ScoringMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public final SAT m_SAT = new SAT();
    public final Intake m_intake = new Intake();
    public final climber m_climber = new climber();
    //public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_pipeline");

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("marker1", Commands.print("Finished 1 Piece"));
            put("marker2", Commands.print("Finished 3-4 Piece"));
            put("Start Intake", new CommandIntakeStart(m_intake));
            put("Start Index", new CommandIndexStart(m_intake));
            put("Stop Intake", new CommandIntakeStop(m_intake));
            put("Stop Index", new CommandIndexStop(m_intake));
            put("Start Shooter", new CommandShootNote(m_SAT, ScoringMode.SUB));
            put("Stop Shooter", new CommandStopShooter(m_SAT));
            put("Intake Note", new SequentialCommandGroup(
                new CommandPivotHandoffPosition(m_SAT),
                new CommandIntakeStart(m_intake),
                new CommandIndexStart(m_intake),
                new CommandIntakeWaitForNote(m_intake),
                // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
                new WaitCommand(0.2), // This just worked more reliably and more easily than the sensor did
                new CommandIntakeStop(m_intake),
                new CommandIndexStop(m_intake),
                new CommandMovePivotToPosition(m_SAT, ScoringMode.SUB)
            ));
            put("Fire From Sub", new SequentialCommandGroup(
                new CommandMovePivotToPosition(m_SAT, ScoringMode.SUB),
                new CommandIndexMoveNoteToFiringPosition(m_intake),
                new CommandShootNote(m_SAT, ScoringMode.SUB),
                new CommandIndexStart(m_intake),
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                    new CommandIndexStop(m_intake),
                    new CommandStopShooter(m_SAT)
                )
            ));
            put("Fire From Podium", new SequentialCommandGroup(
                new CommandMovePivotToPosition(m_SAT, ScoringMode.PODIUM),
                new CommandIndexMoveNoteToFiringPosition(m_intake),
                new CommandShootNote(m_SAT, ScoringMode.PODIUM),
                new CommandIndexStart(m_intake),
                new WaitCommand(0.3),
                new ParallelCommandGroup(
                    new CommandIndexStop(m_intake),
                    new CommandStopShooter(m_SAT)
                )
            ));
        }  
    };

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> driver.leftBumper().getAsBoolean()
            )
        );

        NamedCommands.registerCommands(autonomousCommands);

        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void configureButtonBindings() {
        driverControls();
        // operatorControls();
        // manualTesting();
    }

    public void driverControls(){
        driver.start().and(driver.back()).onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));

        // driver.axisGreaterThan(0, 0.1)
        // .or(driver.axisLessThan(0, -0.1))
        // .or(driver.axisGreaterThan(1, 0.1))
        // .or(driver.axisLessThan(1, -0.1))
        // .or(driver.axisGreaterThan(4, 0.1))
        // .or(driver.axisLessThan(4, -0.1))
        // .whileTrue(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -driver.getLeftY(), 
        //         () -> -driver.getLeftX(), 
        //         () -> -driver.getRightX(), 
        //         () -> driver.leftBumper().getAsBoolean()
        //     )
        // ).onFalse(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> 0, 
        //         () -> 0, 
        //         () -> 0, 
        //         () -> driver.leftBumper().getAsBoolean()
        //     ));

    }

    public void operatorControls(){
        operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.WING));
        operator.pov(180).onTrue(new CommandChangeScoringMode(ScoringMode.SUB));
        // operator.pov(270).onTrue(new CommandChangeScoringMode(ScoringMode.AMP));
        operator.pov(90).onTrue(new CommandChangeScoringMode(ScoringMode.PODIUM));

        operator.x()
           .onTrue(
               new SequentialCommandGroup(
                    new CommandPivotHandoffPosition(m_SAT),
                    new CommandIntakeStart(m_intake),
                    new CommandIndexStart(m_intake),
                    new CommandIntakeWaitForNote(m_intake),
                    // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
                    new WaitCommand(0.2), // This just worked more reliably and more easily than the sensor did
                    new CommandIntakeStop(m_intake),
                    new CommandIndexStop(m_intake),
                    new CommandMovePivotToPosition(m_SAT, ScoringConstants.ScoringMode.SUB)
                )
            );

        operator.y()
            .onTrue(
                new SequentialCommandGroup(
                    new CommandMovePivotToPosition(m_SAT), // pivot move to whatever current mode is
                    new CommandIndexMoveNoteToFiringPosition(m_intake),
                    new CommandShootNote(m_SAT),
                    new CommandIndexStart(m_intake),
                    new WaitCommand(0.3),
                    new ParallelCommandGroup(
                        new CommandIndexStop(m_intake),
                        new CommandStopShooter(m_SAT)
                    )
                )
            );

        // operator.y()
        //     .whileTrue(
        //         m_climber.climbDownRightCommand()
        //     );
    
        // operator.x()
        //     .whileTrue(
        //         m_climber.climbUpRightCommand()
        //     );
            
        // operator.a()
        //     .whileTrue(
        //         m_climber.climbUpLeftCommand()
        //     );
    
        // operator.b()
        //     .whileTrue(
        //         m_climber.climbDownLeftCommand()
        //     );
    }

    public void manualTesting(){
        operator.pov(90).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(0.25), m_SAT));
        operator.pov(270).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(-0.25), m_SAT));
        operator.pov(90).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(0.5), m_SAT));
        operator.pov(270).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(-0.5), m_SAT));

        driver.pov(0).onTrue(new CommandIndexStart(m_intake));
        driver.pov(180).onTrue(new CommandIndexStop(m_intake));

        driver.pov(90).onTrue(new CommandIntakeStart(m_intake));
        driver.pov(270).onTrue(new CommandIntakeStop(m_intake));
    }

    public void operatorTesting(){
        /************************/
        /*                      */
        /*   Operator Buttons   */
        /*                      */
        /************************/

        // operator.axisGreaterThan(5, 0.5)
        //     .whileTrue(
        //         m_climber.climbDownCommand()
        //     );

        // operator.axisLessThan(5, -0.5)
        //     .whileTrue(
        //         m_climber.climbUpCommand()
        //     );

        // operator.axisLessThan(5, 0.5)
        //     .whileTrue(
        //         m_climber.climbMotorStop()
        //     );
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