package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandOverrideIndexStart;
import frc.robot.commands.CommandOverrideIndexStop;
import frc.robot.commands.CommandOverrideIntakeStart;
import frc.robot.commands.CommandOverrideIntakeStop;
import frc.robot.commands.CommandShootNote;
import frc.robot.commands.CommandStopShooter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SAT.SAT;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.intake.Intake;

import java.util.HashMap;
import java.util.Map;

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

    /* Commands */
    public CommandOverrideIntakeStart commandOverrideIntakeStart = new CommandOverrideIntakeStart(m_intake);
    public CommandOverrideIndexStart commandOverrideIndexStart = new CommandOverrideIndexStart(m_intake);
    public CommandOverrideIntakeStop commandOverrideIntakeStop = new CommandOverrideIntakeStop(m_intake);
    public CommandOverrideIndexStop commandOverrideIndexStop = new CommandOverrideIndexStop(m_intake);

    public CommandShootNote commandShootNote = new CommandShootNote(m_SAT);
    public CommandStopShooter commandStopShooter = new CommandStopShooter(m_SAT);

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);
    
    // public CommandBasesPosition commandGoToBasePodiumPosition = new CommandBasesPosition("Podium", m_SAT);
    // public CommandBasesPosition commandGoToBaseSubPosition = new CommandBasesPosition("Sub", m_SAT);
    // public CommandBasesPosition commandGoToBaseTrapPosition = new CommandBasesPosition("Trap", m_SAT);
    // public CommandBasesPosition commandGoToBaseZeroPosition = new CommandBasesPosition("Zero", m_SAT);
    // public CommandBasesPosition commandGoToBaseAmpPosition = new CommandBasesPosition("Amp", m_SAT);
    // public CommandBasesPosition commandGoToBaseWingPosition = new CommandBasesPosition("Wing", m_SAT);
    // public CommandPivotPosition commandGoToPivotPodiumPosition = new CommandPivotPosition("Podium", m_SAT);
    // public CommandPivotPosition commandGoToPivotSubPosition = new CommandPivotPosition("Sub", m_SAT);
    // public CommandPivotPosition commandGoToPivotTrapPosition = new CommandPivotPosition("Trap", m_SAT);
    // public CommandPivotPosition commandGoToPivotZeroPosition = new CommandPivotPosition("Zero", m_SAT);
    // public CommandPivotPosition commandGoToPivotAmpPosition = new CommandPivotPosition("Amp", m_SAT);
    // public CommandPivotPosition commandGoToPivotWingPosition = new CommandPivotPosition("Wing", m_SAT);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("Start Intake", commandOverrideIntakeStart);
            put("Start Index", commandOverrideIndexStart);
            put("Stop Intake", commandOverrideIntakeStop);
            put("Stop Index", commandOverrideIndexStop);
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

        // Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Finished 1 Piece"));
        NamedCommands.registerCommand("marker2", Commands.print("Finished 3-4 Piece"));

        NamedCommands.registerCommands(autonomousCommands);

        // NamedCommands.registerCommand("Warm Up Shooter", Commands.runOnce(() -> m_SAT.shootNote(), m_SAT));

        // NamedCommands.registerCommand("Go To Base Podium Positon", Commands.runOnce(() -> m_SAT.goToBasePodiumPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base AMP Positon", Commands.runOnce(() -> m_SAT.goToBaseAmpPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Sub Positon", Commands.runOnce(() -> m_SAT.goToBaseSubPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Trap Positon", Commands.runOnce(() -> m_SAT.goToBaseTrapPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Base Zero Positon", Commands.runOnce(() -> m_SAT.goToBaseZeroPosition(), m_SAT));

        // NamedCommands.registerCommand("Go To Pivot Podium Positon", Commands.runOnce(() -> m_SAT.goToPivotPodiumPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot AMP Positon", Commands.runOnce(() -> m_SAT.goToPivotAmpPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Sub Positon", Commands.runOnce(() -> m_SAT.goToPivotSubPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Trap Positon", Commands.runOnce(() -> m_SAT.goToPivotTrapPosition(), m_SAT));
        // NamedCommands.registerCommand("Go To Pivot Zero Positon", Commands.runOnce(() -> m_SAT.goToPivotZeroPosition(), m_SAT));
         
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public void configureButtonBindings() {
        /************************/
        /*                      */
        /*    Driver Buttons    */
        /*                      */
        /************************/
        
        // driver.y()
        //    .onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));

        // driver.a()
        //     .onTrue(commandSwerveToNote.alongWith(m_intake.collectNote()));

        // driver.leftTrigger()
        //     .onTrue(Commands.runOnce(() -> m_SAT.shootNote(), m_SAT));
        // driver.rightTrigger()
        //     .onTrue(Commands.runOnce(() -> m_SAT.stopShooter(), m_SAT));

        driver.leftBumper()
             .onTrue(Commands.run(() -> m_SAT.shootNote(), m_SAT));

        driver.rightBumper()
             .onTrue(Commands.run(() -> m_SAT.stopShooter(), m_SAT));

        // driver.pov(0).onTrue(commandOverrideIntakeStart);
        // driver.pov(180).onTrue(commandOverrideIntakeStop);
        driver.pov(90).onTrue(commandOverrideIndexStart);
        driver.pov(270).onTrue(commandOverrideIndexStop);


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

        // operator.axisGreaterThan(5, -0.5)
        //     .whileTrue(
        //         m_climber.climbMotorStop()
        //     );
            
        // operator.axisGreaterThan(1, 0.5)
        //     .whileTrue(
        //         m_climber.climbMidLeftCommand()
        //     );

        // operator.axisLessThan(1, -0.5)
        //     .whileTrue(
        //         m_climber.climbMidRightCommand()
        //     );

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

        operator.pov(0).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(0.25), m_SAT));
        operator.pov(180).whileTrue(new RunCommand(() -> m_SAT.pivotGoToPositionIncrement(-0.25), m_SAT));
        operator.pov(90).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(0.5), m_SAT));
        operator.pov(270).whileTrue(new RunCommand(() -> m_SAT.baseGoToPositionIncrement(-0.5), m_SAT));

        // // RUN STEP 1 OF SUB
        // operator.x().onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.SUB, true, false, false, 0));
        // // RUN STEP 2 OF SUB`
        // operator.a().onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.SUB, false, true, false, 0));
        // // RUN STEP 3 OF SUB
        // operator.b().onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.SUB, false, false, true, 0));
        // // RUN ALL STEPS OF SUB
        // operator.y().onTrue(m_SAT.moveSATToPosition(Constants.SATConstants.Position.SUB, 1));

        // // RUN STEP 1 OF START
        // operator.pov(270).onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.START, true, false, false, 0));
        // // RUN STEP 2 OF START
        // operator.pov(180).onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.START, false, true, false, 0));
        // // RUN STEP 3 OF START
        // operator.pov(90).onTrue(m_SAT.moveSAT(Constants.SATConstants.Position.START, false, false, true, 0));
        // // RUN ALL STEPS OF START
        // operator.pov(0).onTrue(m_SAT.moveSATToPosition(Constants.SATConstants.Position.START, 1));

        //operator.rightBumper()
           // .onTrue(m_intake.collectNote());

        //PHASE 2 TESTING, INTAKE TO INDEXER
        operator.rightBumper()
            .onTrue(commandOverrideIndexStop);

        operator.leftBumper()
            .onTrue(commandOverrideIntakeStart);

        operator.leftTrigger()
            .onTrue(commandOverrideIntakeStop);

        operator.a()
            .onTrue(m_intake.passNoteToIndex());

        //PHASE 3 TESTING, INDEXER TO SHOOTER
        operator.b()
        .onTrue(commandOverrideIndexStop);

        operator.x()
        .onTrue(commandStopShooter);

        operator.y()
        .onTrue(new SequentialCommandGroup(commandShootNote,m_intake.fireNote(), commandStopShooter));



        //operator.start()
        //    .whileFalse(commandOverrideIntakeStop.andThen(commandOverrideIndexStop));
        
        // if (m_intake.simulationDebugMode) {
        //     operator.a()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(true);}));
        //     operator.b()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(true);}));
        //     operator.x()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setLowerSensorDetectsNote(false);}));
        //     operator.y()
        //         .onTrue(Commands.runOnce(() -> {m_intake.setUpperSensorDetectsNote(false);}));
        // }
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