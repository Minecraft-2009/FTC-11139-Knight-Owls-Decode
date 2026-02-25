package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Storage {
    public static Pose autoEndPose;

    public void setAutoEndPose(Pose pose) {
        autoEndPose = pose;
    }

    public static Pose getAutoEndPose() {
        return autoEndPose;
    }
}
