package org.firstinspires.ftc.teamcode.util;

/**
 * Defines the possible states of the UDP listener.
 * This is a standalone enum to be used by both ListenForCurvePoints and ListenResult.
 */
public enum ListenStatus {
    NOT_STARTED,
    LISTENING,
    COMPLETED_SUCCESSFULLY,
    TIMED_OUT,
    STOPPED,
    ERROR
}
