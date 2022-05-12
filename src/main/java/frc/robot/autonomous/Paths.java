package frc.robot.autonomous;

import java.util.Arrays;

import com.ma5951.utils.autonomous.Path;
import com.ma5951.utils.autonomous.Waypoint;

public class Paths {
        /**
         * Autonomous Paths
         */
        public static final Path getingOutOfLunchPadPart1 = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 1.3)), 0.2, 1.5, 4, 2.7, 0.2, 0.02);

        public static final Path getingOutOfLunchPadPart2 = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 1.2)), 0.2, 1.5, 3, 10, 0.2, 0.02);

        public static final Path goToTheSecondBallPart1 = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(-0.3, 0.4), new Waypoint(-0.6, 0.3),
                                        new Waypoint(-0.9, 0)),
                        0.2, 3, 3.7, 1.5, 0.3, 0.02);
        public static final Path goToTheSecondBallPart2 = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0.25, 2.5)), 0.1, 1.2, 2.8, 1.2, 0.3, 0.04);

        public static final Path goToTheSecondBallPart3 = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2), new Waypoint(-0.5, 1.75)), 0.2, 1, 3, 1,
                        0.3, 0.035);

        public static final Path goToHPBall = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(1.25, 3.25)), 0.2, 0.85, 2.6, 1,
                        0.3, 0.01);

        public static final Path goToHPBallFrom3Ball = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 0.2), new Waypoint(2, 3)), 0.2, 0.85, 2.8,
                        1,
                        0.31, 0.026);

        public static final Path goToShootingPosition = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2)), 0.2, 1.5, 2.8, 15,
                        0.5, 0.045);

        public static final Path goToShootingPosition5Balls = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2.75)), 0.2, 1.5, 2.8, 15,
                        0.5, 0.045);

        public static final Path goTo5thBall = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(-0.15, 2)), 0.2, 1.5, 3.8, 15,
                        0.5, 0.045);


        public static final Path gettingOutOfLunchPad = new Path(
                        Arrays.asList(new Waypoint(0, 0), new Waypoint(0, 2.2)), 0.1, 1.5, 3.8, 15, 0.5, 0.045);

}
