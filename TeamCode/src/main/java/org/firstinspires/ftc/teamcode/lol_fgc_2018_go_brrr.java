package org.firstinspires.ftc.teamcode;

import android.app.ActivityManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.concurrent.ConcurrentLinkedDeque;

import static java.lang.Math.abs;

@TeleOp

public class lol_fgc_2018_go_brrr extends LinearOpMode {
    private AnalogInput sensorfarleft, sensorfarright, sensornearright, sensornearleft;
    private int snfl=0, snfr=0, snnr=0, snnl=0, error = 0;
    private double p = 0.0, i = 0.0, d = 0.0, powerCorectie = 0.0,con=0.21,conpow=0.05,conpid = 0.001,forward,right,clockwise,ss,ds,df,sf,max,systime;
    private DcMotorEx motorss,motords,motordf,motorsf;
    private Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    private boolean active = false;
    @Override
    public void runOpMode() throws InterruptedException {
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");
        sensorfarleft = hardwareMap.get(AnalogInput.class, "snfl");
        sensorfarright = hardwareMap.get(AnalogInput.class, "snfr");
        sensornearright = hardwareMap.get(AnalogInput.class, "snnr");
        sensornearleft = hardwareMap.get(AnalogInput.class, "snnl");

        motordf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorsf.setDirection(DcMotorEx.Direction.REVERSE);
        motorss.setDirection(DcMotorEx.Direction.REVERSE);
        //motordf.setDirection(DcMotorEx.Direction.REVERSE);
        //motords.setDirection(DcMotorEx.Direction.REVERSE);

        motordf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pid.setSetpoint(0);
        pid.enable();
        waitForStart();
        PID.start();
        gamepads.start();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("P:", p);
            telemetry.addData("I:", i);
            telemetry.addData("D:", d);
            telemetry.addData("Error:", error);
            telemetry.addData("PowerCorectie:", powerCorectie);
            telemetry.addData("SFR", sensorfarright.getVoltage());
            telemetry.addData("SNR", sensornearright.getVoltage());
            telemetry.addData("SNL", sensornearleft.getVoltage());
            telemetry.addData("SFL", sensorfarleft.getVoltage());
            telemetry.addData("SNFR", snfr);
            telemetry.addData("SNNR", snnr);
            telemetry.addData("SNNL", snnl);
            telemetry.addData("SNFL", snfl);
            telemetry.addData("motords", motords.getCurrentPosition());
            telemetry.addData("motorss", motorss.getCurrentPosition());
            telemetry.addData("motordf", motordf.getCurrentPosition());
            telemetry.addData("motorsf", motorsf.getCurrentPosition());
            telemetry.update();
        }
    }
    private Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            double lastp = 0, lasti = 0, lastd = 0;
            while(!isStopRequested() && opModeIsActive()){
                if(sensorfarleft.getVoltage()<con){

                    snfl = -10;
                }
                else{
                    snfl = 0;
                }
                if(sensorfarright.getVoltage()<con){
                    snfr = 10;
                }
                else{
                    snfr = 0;
                }
                if(sensornearleft.getVoltage()<con){
                    snnl = -1;
                }
                else{
                    snnl = 0;
                }
                if(sensornearright.getVoltage()<con){
                    snnr = 1;
                }
                else{
                    snnr = 0;
                }
                error = snfl + snnl + snnr + snfr;


                pid.setPID(p,i,d);
                powerCorectie = pid.performPID(error);
                if(gamepad1.right_bumper){
                    active = false;
                }
                else if(gamepad1.left_bumper){
                    active = true;
                }
                systime = System.currentTimeMillis();
                if(active){
                    if(Math.abs(error) > 2){
                        if(snnr != 0 && snnl != 0){
                            while(systime + 850 > System.currentTimeMillis()){
                                setPower(0.1,0.1,0.1,0.1);
                            }
                            systime = System.currentTimeMillis();
                            if(snfl!=0) {
                                while (systime + 1950 > System.currentTimeMillis()) {
                                    setPower(-0.1, 0.1, -0.1, 0.1);
                                }
                            }
                            else if(snfr!=0){
                                while (systime + 1940 > System.currentTimeMillis()) {
                                    setPower(0.1, -0.1, 0.1, -0.1);
                                }
                            }
                        }
                        setPower(powerCorectie, powerCorectie, powerCorectie, powerCorectie);
                    }
                    else{
                        setPower(conpow + powerCorectie, conpow - powerCorectie, conpow + powerCorectie, conpow - powerCorectie);
                    }
                }
                else{
                    forward = gamepad1.left_stick_y;
                    right = -gamepad1.left_stick_x;
                    clockwise = gamepad1.right_stick_x;


                    /**calculating the power for motors */
                    df = -forward - clockwise + right;
                    ss = -forward + clockwise + right;
                    sf = -forward + clockwise - right;
                    ds = -forward - clockwise - right;

                    /**normalising the power values*/
                    max = abs(sf);
                    if (abs(df) > max) {
                        max = abs(df);
                    }
                    if (abs(ss) > max) {
                        max = abs(ss);
                    }
                    if (abs(ds) > max) {
                        max = abs(ds);
                    }
                    if (max > 1) {
                        sf /= max;
                        df /= max;
                        ss /= max;
                        ds /= max;
                    }
                    setPower(df/5,sf/5,ds/5,ss/5);
                }

            }
        }
    });
    private Thread gamepads = new Thread(new Runnable() {
        @Override
        public void run() {
            long lastTime = System.currentTimeMillis();
            while(!isStopRequested() && opModeIsActive()){
                if(gamepad1.a && System.currentTimeMillis() - 200 > lastTime){
                    p-=conpid;
                    lastTime = System.currentTimeMillis();
                }
                else if(gamepad1.y && System.currentTimeMillis() - 200 > lastTime){
                    p+=conpid;
                    lastTime = System.currentTimeMillis();
                }

                if(gamepad1.x && System.currentTimeMillis() - 200 > lastTime){
                    i-=conpid;
                    lastTime = System.currentTimeMillis();
                }
                else if(gamepad1.b && System.currentTimeMillis() - 200 > lastTime){
                    i+=conpid;
                    lastTime = System.currentTimeMillis();
                }

                if(gamepad1.dpad_down && System.currentTimeMillis() - 200 > lastTime){
                    d-=conpid;
                    lastTime = System.currentTimeMillis();
                }
                else if(gamepad1.dpad_up && System.currentTimeMillis() - 200 > lastTime){
                    d+=conpid;
                    lastTime = System.currentTimeMillis();
                }
            }
        }
    });
    private void setPower(double df, double sf, double ds, double ss){
        motordf.setPower(df);
        motorsf.setPower(sf);
        motords.setPower(ds);
        motorss.setPower(ss);
    }
}
