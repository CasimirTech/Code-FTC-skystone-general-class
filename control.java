package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class control{


    public static DcMotor Drive_LF = null;
    public static DcMotor Drive_LB = null;
    public static DcMotor Drive_RF = null;
    public static DcMotor Drive_RB = null;
    static HardwareMap hwMap =  null;

    final static double counts_per_rev = 1120;
    final static double sprocket_ratio = 15/20; //motor shaft connected to 20 teeth and wheel connected to 15 teeth
    final static double wheel_diameter_cm = 10;
    final static double wheel_circumference = wheel_diameter_cm * Math.PI;

    //variables needed for TensorFlow
    public static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";
    public static final String VUFORIA_KEY =
            " AVgBSO3/////AAABmUABgkWe3k5ft8ovIqEzuJsHwi7jr9LLpv+7JFkEGeDgtkSLJrFBmFceTUqHF3cIADKtRGCjHjpJjdeYXvJgXGB6u9htsM09eLMYcLg45Htjsa7X0xNvqYpU1kNduJyeYsiA722/vC2v7GmQfT3EIxzxhIEc9Ct5vrJTLN9zWLgWxTMgj8lTgmnm7OPsO/ctejJp1T4pF0y3sGeHqCMGvnOjApUIHm3eqqecs0r/SXUsYMH/iSgvMIJ7o9zbVeFNKGRxWmOBg5Wjjv788FjiZSJ+Iw1+AJVxBE5Mpywnc8pExJY9sOoj35HyDLogTzxA/s0jahhvG9y0nJ3E+GDnmWPIIQt0TXOS/AR2IF8GBIC5";
    public static VuforiaLocalizer vuforia;
    public static TFObjectDetector tfod;

    /**=======================================================================================================*/
    /** CONSTANTS USED IN TELEOP AND AUTONOMOUS */
    // Heading of the robot(R) relative to the heading of the driver(D) (driverHeading = 0 degrees) at the start of the match
    // Clockwise is negative and counterclockwise is positive
    public static double startDeltaHeading = 90;
    final static double drive_speed = 0.6;
    final static double timeout_skystone_search = 5000;
    /**=======================================================================================================*/


    public static void initialize(HardwareMap ahwMap) {

        /**
        INITIALIZING MOTORS
         */

        hwMap = ahwMap;

        // Initialize hardware variables for the motors on the robot
        Drive_LF = hwMap.get(DcMotor.class, "left_front");
        Drive_RF = hwMap.get(DcMotor.class, "right_front");
        Drive_LB = hwMap.get(DcMotor.class, "left_back");
        Drive_RB = hwMap.get(DcMotor.class, "right_back");

        // Left wheels need to rotate counter clockwise for the robot to move forward
        // Reversing the left motors will make them turn counter clockwise when assigned a power of 1
        Drive_LB.setDirection(DcMotor.Direction.REVERSE);
        Drive_LF.setDirection(DcMotor.Direction.REVERSE);
        Drive_RF.setDirection(DcMotor.Direction.FORWARD);
        Drive_RB.setDirection(DcMotor.Direction.FORWARD);

        Drive_LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive_LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive_RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Drive_RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Drive_LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * INITIALIZE TENSORFLOW AND VUFORIA
         */
        //initialize vuforia for object detection
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            //telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
    AUTONOMOUS FUNCTIONS
     */

    public void driveT(double V_Des, double Theta_D, int time) throws InterruptedException {

        double[] power = new double[4];
        power[0] = V_Des * Math.sin(Theta_D + Math.PI / 4);
        power[1] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[2] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[3] = V_Des * Math.sin(Theta_D + Math.PI / 4);

        setPower(power);
        int teller = 0;
        while(teller < time){
            wait(1);
            teller ++;
        }
        stopDrive();
    }
    public static void driveEnc(double V_Des, double Theta_D, int dist_cm){

        double newTarget = ((dist_cm / wheel_circumference)  * sprocket_ratio) * counts_per_rev;
        int target = (int) newTarget;

        Drive_LF.setTargetPosition(target);
        Drive_LB.setTargetPosition(target);
        Drive_RB.setTargetPosition(target);
        Drive_RF.setTargetPosition(target);

        Drive_LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drive_LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drive_RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Drive_RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double[] power = new double[4];
        power[0] = V_Des * Math.sin(Theta_D + Math.PI / 4);
        power[1] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[2] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[3] = V_Des * Math.sin(Theta_D + Math.PI / 4);

        setPower(power);

        while(ifAllBusy()){}

        stopDrive();

    }
    public static void driveUnl(double V_Des, double Theta_D){

        double[] power = new double[4];
        power[0] = V_Des * Math.sin(Theta_D + Math.PI / 4);
        power[1] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[2] = V_Des * Math.cos(Theta_D + Math.PI / 4);
        power[3] = V_Des * Math.sin(Theta_D + Math.PI / 4);

        setPower(power);


    }

    public void turnGyr(double V_Theta, double angle){

    }



    /**
    GENERAL FUNCTIONS
     */

    static void setPower(double[] power){
        Drive_LF.setPower(power[0]);
        Drive_RF.setPower(power[1]);
        Drive_LB.setPower(power[2]);
        Drive_RB.setPower(power[3]);
    }

    public static void stopDrive(){
        Drive_LF.setPower(0);
        Drive_RF.setPower(0);
        Drive_LB.setPower(0);
        Drive_RB.setPower(0);

        Drive_LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Drive_RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    static boolean ifAllBusy(){
        return  Drive_LF.isBusy() &&
                Drive_LB.isBusy() &&
                Drive_RB.isBusy() &&
                Drive_RF.isBusy();
    }

    /**
     VUFORIA/TENSORFLOW FUNCTIONS
     */


    public static void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    //initialize the TensorFlow object
    public static void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    static Recognition getSkystone(){
        //get the list of recognised objects (stones and skystones)
        List<Recognition> recognitions = tfod.getRecognitions();
        //check if any objects are detected
        if (recognitions != null) {
            //cycle trough the list of objects and check for skystones
            for (Recognition recognition : recognitions) {
                if (recognition.getLabel() == "Skystone"){
                    return recognition;
                }
            }
        }
        return null;
    }

}