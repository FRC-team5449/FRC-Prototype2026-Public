package frc.robot.subsystems.index;

public class IndexConstants {
    public static int indexMotorCanId = 21;
    public static String indexCanBus = "rio";

    public static double stallCurrentThreshold = 20.0;
    public static double stallCooldownTime = 0.1;
    public static int stallCyclesRequired = 5;
    public static double stallRecoveryGracePeriod = 0.2;
}