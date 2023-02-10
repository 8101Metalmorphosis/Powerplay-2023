package org.firstinspires.ftc.teamcode.other;

public class Constants {

    public static class DriveConstants {
        public static float speedLimiter = 1.2f;
    }

    public static class ArmConstants {
        public static int maxArmPosition = 3500;
        public static int lowArmPosition = -5;

        public static int highJuncArmPosition = 3400; // 3266
        public static int midJuncArmPosition = 2500; // 2353
        public static int lowJuncArmPosition = 1500; // 1329

        public static int armThreshold = 650;
        public static int armInterval = 500;

        public static int armPower = 1;
    }

    public static class TurnTableConstants {
        public static int TURNTABLE_FRONT = 0;
        public static int TURNTABLE_LEFT = -648;
        public static int TURNTABLE_RIGHT = 648;
        public static int TURNTABLE_BACK = -1280;

        public static int turnTableInterval = 300;
    }

    public static class ClawConstants {
        public static double closePosition = .25;
        public static double openPosition = 1;
    }

    public static class OtherConstants {

    }

//    (if)sin city
//        {enter} not made for you
}