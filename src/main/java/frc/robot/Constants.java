/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;
	public static final int Xbobcontroller = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	public static final int kTimeoutMs = 30;

    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

    public static final class ArmProfile {
      /* Arm ID's */
      public static final int LEFT_OUTER_ARM = 15; //FIXME
      public static final int RIGHT_OUTER_ARM = 18; //FIXME
      public static final int LEFT_INNER_ARM = 17; //FIXME
      public static final int RIGHT_INNER_ARM = 14; //FIXME
      public static final int WRIST_MOTOR = 28; //FIXME

	  public static final int CRUISE_VEL = 15000; //raw sensor unit
	  public static final int MOTION_ACCEL = 6000; //raw sensor unit

  }

  /* Falcon counts per rotation */
  public static final int FALCON_ENCODER_COUNTS = 2048;

  
}