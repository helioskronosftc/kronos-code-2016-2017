package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class KronosHardware
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  uber_liftMotor    = null;
    public DcMotor  launcherMotor    = null;
    public Servo    servoL1         = null;
    public Servo    servoR2         = null;
    public Servo    servo_launch         = null;
    public Servo    servo_flag         = null;
    public ColorSensor sensorRGB;
    public DeviceInterfaceModule cdim;
    public OpticalDistanceSensor distanceSensor1;
    public OpticalDistanceSensor distanceSensor2;
    static final int LED_CHANNEL = 5;

    //  public Servo    leftClaw    = null;
  //  public Servo    rightClaw   = null;

  //  public static final double MID_SERVO       =  0.5 ;
  //  public static final double ARM_UP_POWER    =  0.45 ;
  //  public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public KronosHardware(){

    }

    /*  Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        sensorRGB = hwMap.colorSensor.get("color");
        cdim = hwMap.deviceInterfaceModule.get("cdim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        distanceSensor1 = hwMap.opticalDistanceSensor.get("dist1");
        distanceSensor2 = hwMap.opticalDistanceSensor.get("dist2");

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        uber_liftMotor  = hwMap.dcMotor.get("uber_lift");
        launcherMotor   = hwMap.dcMotor.get("lunch");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uber_liftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        uber_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        uber_liftMotor.setPower(0);
        launcherMotor.setPower(0);

        cdim.setDigitalChannelState(LED_CHANNEL, false);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uber_liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        servoL1 = hwMap.servo.get("servo_L1");
        servoR2 = hwMap.servo.get("servo_R2");
        servo_launch = hwMap.servo.get("servo_launch");
        servo_flag = hwMap.servo.get("servo_flag");
        //    leftClaw = hwMap.servo.get("left_hand");
    //    rightClaw = hwMap.servo.get("right_hand");
        servoL1.setPosition(0.5);
        servoR2.setPosition(0.0);
        servo_launch.setPosition(0.5);
        servo_flag.setPosition(0.0);
    //    leftClaw.setPosition(MID_SERVO);
    //    rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

