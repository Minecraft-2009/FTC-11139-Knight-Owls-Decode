package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightCode extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0); // april tag detection

        /*
         * Starts polling for data.
         */
        limelight.start();

        while (opModeIsActive()) {

            // Start detection for balls
            // Assuming you have a Limelight instance or are using the static helpers
// The parameter "auto_pov_10s" is the desired name for your snapshot
//            limelight.captureSnapshot("live_feed");

            LLResult result;


            //happens after april tag motif detection
            limelight.pipelineSwitch(1);

            double PLtx, PLty, PLta, PCtx, PCty, PCta, PRtx, PRty, PRta, GLtx, GLty, GLta, GCtx, GCty, GCta, GRtx, GRty, GRta, TLta, TCta, TRta, TLtx, TCtx, TRtx, TLty, TCty, TRty;


            //// happens only once


            //get ta for Purple Left
            result = limelight.getLatestResult(); // pipe 1
            PLtx = result.getTy();
            PLty = result.getTx();
            PLta = result.getTa();


            //get ta for Purple Center
            limelight.pipelineSwitch(2);
            sleep(50);

            result = limelight.getLatestResult(); // pipe 2
            PCtx = result.getTy();
            PCty = result.getTx();
            PCta = result.getTa();


            //get ta for Purple Right
            limelight.pipelineSwitch(3);
            sleep(50);

            result = limelight.getLatestResult(); // pipe 3
            PRtx = result.getTy();
            PRty = result.getTx();
            PRta = result.getTa();


            //get ta for Green Left
            limelight.pipelineSwitch(4);
            sleep(50);

            result = limelight.getLatestResult(); // pipe 4
            GLtx = result.getTy();
            GLty = result.getTx();
            GLta = result.getTa();


            //get ta for Green Center
            limelight.pipelineSwitch(5);
            sleep(50);

            result = limelight.getLatestResult(); // pipe 5
            GCtx = result.getTy();
            GCty = result.getTx();
            GCta = result.getTa();


            //get ta for Green Center
            limelight.pipelineSwitch(6);
            sleep(50);

            result = limelight.getLatestResult(); // pipe 6
            GRtx = result.getTy();
            GRty = result.getTx();
            GRta = result.getTa();


            TLtx = PLtx + GLtx;
            TCtx = PCtx + GCtx;
            TRtx = PRtx + GRtx;

            TLty = PLty + GLty;
            TCty = PCty + GCty;
            TRty = PRty + GRty;

            TLta = PLta + GLta;
            TCta = PCta + GCta;
            TRta = PRta + GRta;

            if (TLta >= 2*TRta) { // case: Left much greater than Right (normally no right)
                if (TCtx >= 0) { ///// TCtx is on the right
                    if (TLta >= TCta) { // case: More on left than in center-right
                        //TODO: Go to Left X
                    }
                }

                else if (TCtx < 0) { ///// TCtx is on the left
                    if (TCtx < -10) { // TCtx is very left
                        //TODO: Go to (TLtx+TCtx)/2 for average - weighted by ta?
                    }
                    else if (TLtx > -15){ //TLtx is right
                        //TODO: Go to (TLtx+TCtx)/2 for average - weighted by ta?
                    }
                    else { // Neither: TLtx is far left, and TCtx is center-left
                        if (TLta >= TCta) {
                            //TODO: Go to Left X
                        }
                        else {
                            //TODO: Go to Center-Left X
                        }
                    }
                }

            }

            else if (TRta >= 2*TLta) { // case: Right much greater than Left (normally no left)
                if (TCtx <= 0) { ///// TCtx is on the left
                    if (TLta >= TCta) { // case: More on right than in center-left
                        //TODO: Go to Right X
                    }
                }

                else if (TCtx > 0) { ///// TCtx is on the right
                    if (TCtx > 10) { // TCtx is very right
                        //TODO: Go to (TRtx+TCtx)/2 for average - weighted by ta?
                    }
                    else if (TRtx < 15){ //TLtx is left
                        //TODO: Go to (TRtx+TCtx)/2 for average - weighted by ta?
                    }
                    else { // Neither: TLtx is far right, and TCtx is center-right
                        if (TLta >= TCta) {
                            //TODO: Go to Right X
                        }
                        else {
                            //TODO: Go to Center-Right X
                        }
                    }
                }

            }

            else if ((TCta > TLta) && (TCta > TRta)) { // Center much greater than either side
                //TODO: Go to Center X i guess
            }

            else if ((TLta > 1.2*TCta) && (TRta > 1.2*TCta)) { // Center does not have balls compared to sides
                if (TLta > 1.2*TRta) { // Left has more than Right
                    //TODO: Go to Left X
                }
                else if (TRta > 1.2*TLta) { // Right has more than Left
                    //TODO: Go to Right X
                }
                else { // very similar sizes

                }
            }



            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }
}