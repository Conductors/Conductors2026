package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Drivetrain;


public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Drivetrain m_drivetrain;

    // Auto strings - convert the choreo file names to variables
    private final String centerBack = "center_backup";
    private final String centerLeftSwerve = "center_swerve_left";
    //...and so on

    public AutoRoutines(AutoFactory factory, Drivetrain drivetrain) {
        m_factory = factory;
        m_drivetrain = drivetrain;
    }

    public AutoRoutine centerScore() {

        AutoRoutine routine = m_factory.newRoutine("Center Score Then Left");

        AutoTrajectory centerBackTraj = routine.trajectory(centerBack);
        AutoTrajectory centerLeftSwerveTraj = routine.trajectory(centerLeftSwerve);

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(centerBack), new ScheduleCommand(untangle()), centerBackTraj.cmd()));

        centerLeftSwerveTraj
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        

        return routine;
    }

    public AutoRoutine oneCoralRightRoutine() {
        AutoRoutine routine = m_factory.newRoutine("Three Coral Right");

        AutoTrajectory startToFirstCoral = routine.trajectory(rightStart);
        AutoTrajectory firstCoralToHp = routine.trajectory(backRightRightToHp);
        AutoTrajectory hpToSecondCoral = routine.trajectory(hpToFrontRight);
        AutoTrajectory secondCoralToHp = routine.trajectory(frontRightLeftToHp);
        AutoTrajectory hpToThirdCoral = routine.trajectory(hpToFrontRight);
        AutoTrajectory thirdCoralToHp = routine.trajectory(frontRightRightToHp);
        AutoTrajectory hpToFourthCoral = routine.trajectory(hptoBackRight);

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(rightStart), new ScheduleCommand(untangle()), startToFirstCoral.cmd()));

        hpToSecondCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToThirdCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));
        hpToFourthCoral
                .active()
                .and(RobotContainer.getCoralScore()::hasCoral)
                .onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));

        startToFirstCoral
                .atTime(0.35)
                .onTrue(Commands.sequence(
                        new ScheduleCommand(
                                RobotContainer.getFunnelTilt().goToPosition(() -> FunnelTiltConstants.kIntakePosition)),
                        aimRight(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimRight(false)),
                        firstCoralToHp.cmd().asProxy()));
        firstCoralToHp.atTime(1).onTrue(intake());
        firstCoralToHp.chain(hpToSecondCoral);
        hpToSecondCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitSeconds(0.1),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondCoralToHp.cmd().asProxy()));
        secondCoralToHp.atTime(1).onTrue(intake());
        secondCoralToHp.chain(hpToThirdCoral);
        hpToThirdCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimRight(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimRight(false)),
                        thirdCoralToHp.cmd().asProxy()));
        thirdCoralToHp.atTime(1).onTrue(intake());
        thirdCoralToHp.chain(hpToFourthCoral);
        hpToFourthCoral
                .atTimeBeforeEnd(0.5)
                .onTrue(Commands.sequence(
                        aimLeft(),
                        Commands.waitSeconds(0.15),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        Commands.waitUntil(() -> RobotContainer.getWrist().atL4())
                                .withTimeout(0.5),
                        score()));

        return routine;

        // routine.active()
        //         .onTrue(Commands.sequence(
        //                 m_factory.resetOdometry(rightStart), new ScheduleCommand(untangle()),
        // startToFirstCoral.cmd()));

        // startToFirstCoral.active().onTrue(RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.L3));

        // startToFirstCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimLeft(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 firstCoralToHp.cmd().asProxy()));
        // firstCoralToHp.atTime(1).onTrue(intake());
        // firstCoralToHp.chain(hpToSecondCoral);
        // hpToSecondCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimRight(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 secondCoralToHp.cmd().asProxy()));
        // secondCoralToHp.atTime(1).onTrue(intake());
        // secondCoralToHp.chain(hpToThirdCoral);
        // hpToThirdCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimLeft(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score(),
        //                 thirdCoralToHp.cmd().asProxy()));
        // thirdCoralToHp.atTime(1).onTrue(intake());
        // thirdCoralToHp.chain(hpToFourthCoral);
        // hpToFourthCoral
        //         .recentlyDone()
        //         .onTrue(Commands.sequence(
        //                 aimRight(),
        //                 Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
        //                 Commands.waitUntil(() -> RobotContainer.getWrist().atL4()),
        //                 score()));

        // return routine;
    }

    public AutoRoutine centerAutoRightFirst() {
        AutoRoutine routine = m_factory.newRoutine("Center Auto Right First");

        AutoTrajectory startToFirst = routine.trajectory(centerAuto, 0);
        AutoTrajectory firstToBarge = routine.trajectory(centerAuto, 1);
        AutoTrajectory bargeToSecond = routine.trajectory(centerAuto, 2);
        AutoTrajectory secondToBarge = routine.trajectory(centerAuto, 3);
        AutoTrajectory bargeToThird = routine.trajectory(centerAuto, 4);
        AutoTrajectory thirdToBarge = routine.trajectory(centerAuto, 5);
        AutoTrajectory bargeBack = routine.trajectory(centerAuto, 6);

        Elevator elevator = RobotContainer.getElevator();
        Wrist wrist = RobotContainer.getWrist();

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(centerStart), new ScheduleCommand(untangle()), startToFirst.cmd()));

        startToFirst
                .atTime(0)
                .onTrue(Commands.sequence(
                        aimLeftl2(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        firstToBarge.cmd().asProxy()));

        firstToBarge
                .atTime(0.6)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        firstToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        algeaScore(),
                        bargeToSecond.cmd().asProxy()));

        bargeToSecond
                .atTimeBeforeEnd(0.8)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondToBarge.cmd().asProxy()));

        secondToBarge
                .atTimeBeforeEnd(0.4)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        secondToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeToThird.cmd().asProxy()));

        bargeToThird
                .atTimeBeforeEnd(0.6)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        thirdToBarge.cmd().asProxy()));

        thirdToBarge
                .atTimeBeforeEnd(0.25)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        thirdToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeBack.cmd().asProxy()));

        return routine;
    }

    public AutoRoutine centerAutoLeftFirst() {

        AutoRoutine routine = m_factory.newRoutine("Center Auto Left First");

        AutoTrajectory startToFirst = routine.trajectory(centerToFirst);
        AutoTrajectory firstBarge = routine.trajectory(firstToBarge);
        AutoTrajectory bargeToSecond = routine.trajectory(bargeToLeft);
        AutoTrajectory secondBarge = routine.trajectory(secondToBarge);
        AutoTrajectory bargeToThird = routine.trajectory(bargeToRight);
        AutoTrajectory thirdToBarge = routine.trajectory(thirdBarge);
        AutoTrajectory bargeBack = routine.trajectory(bargeOffLine);

        startToFirst
                .atTime(0)
                .onTrue(Commands.sequence(
                        aimLeftl2(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        firstBarge.cmd().asProxy()));

        firstBarge
                .atTime(0.6)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        firstBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition())
                                .withTimeout(0.5),
                        algeaScore(),
                        bargeToSecond.cmd().asProxy()));

        bargeToSecond
                .atTimeBeforeEnd(0.8)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        score(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        secondBarge.cmd().asProxy()));

        secondBarge
                .atTimeBeforeEnd(0.4)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        secondBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeToThird.cmd().asProxy()));

        bargeToThird
                .atTimeBeforeEnd(0.6)
                .onTrue(Commands.sequence(
                        aimLeftl3(),
                        Commands.runOnce(() -> RobotStates.setAutoAimLeft(false)),
                        thirdToBarge.cmd().asProxy()));

        thirdToBarge
                .atTimeBeforeEnd(0.25)
                .onTrue(Commands.parallel(
                        RobotContainer.getElevator().setStateAndGoToHeight(ElevatorHeight.Net),
                        RobotContainer.getWrist().net()));

        thirdToBarge
                .recentlyDone()
                .onTrue(Commands.sequence(
                        Commands.waitUntil(RobotContainer.getElevator().isAtPosition()),
                        algeaScore(),
                        bargeBack.cmd().asProxy()));

        routine.active()
                .onTrue(Commands.sequence(
                        m_factory.resetOdometry(centerStart), new ScheduleCommand(untangle()), startToFirst.cmd()));

        return routine;
    }

    /* non-trajectory auto commands */
    public Command aimLeft() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimLeftl3() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol3(true)),
                Commands.runOnce(() -> RobotStates.setAutol3(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimLeftl2() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol2(true)),
                Commands.runOnce(() -> RobotStates.setAutol2(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimLeft(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command aimRight() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutol4(true)),
                Commands.runOnce(() -> RobotStates.setAutol4(false)),
                Commands.runOnce(() -> RobotStates.setAutoAimRight(true)),
                Commands.waitUntil(RobotStates::isAimed));
    }

    public Command score() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitUntil(() -> RobotContainer.getCoralScore().scoredCoral())
                        .withTimeout(0.25),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public Command algeaScore() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotStates.setAutoscore(true)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> RobotStates.setAutoscore(false)));
    }

    public static Command untangle() {
        return Commands.sequence(
                Commands.runOnce(() -> RobotContainer.getClimber().zero()),
                RobotContainer.getClimber().clearFunnel());
    }

    public static Command resetPose() {
        return Commands.runOnce(() -> RobotContainer.getDrivetrain()
                .resetPose(new Pose2d(
                        FieldConstants.fieldLength - Units.inchesToMeters(33.0 / 2.0),
                        FieldConstants.fieldWidth / 2,
                        Rotation2d.k180deg)));
    }

    public void chainWithDelay(AutoTrajectory a, AutoTrajectory b, double delaySeconds) {
        a.doneDelayed(delaySeconds).onTrue(b.cmd());
    }

    public Command nothing(String trajName) {
        return Commands.sequence(m_factory.resetOdometry(trajName), new ScheduleCommand(untangle()));
    }

    public Command intake() {
        return Commands.runOnce(() -> RobotStates.setAutointake(true))
                .andThen(Commands.runOnce(() -> RobotStates.setAutointake(false)));
    }

    public Command nothingRight() {
        return nothing(rightStart);
    }

    public AutoRoutine nothingRightRoutine() {
        AutoRoutine routine = m_factory.newRoutine("Nothing Right");
        routine.active().onTrue(nothingRight());
        return routine;
    }

    public Command nothingLeft() {
        return nothing(leftStart);
    }
}
