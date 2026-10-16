package frc.lib.catalyst.hardware;
import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

class MotorRequestReuseTest {
    @Test void ordinaryPositionRemovesPriorArbitraryFeedforward() {
        assertTrue(HAL.initialize());
        var m=CatalystMotor.builder(32).name("PositionReuse").canBus("can_s0").build();
        try {
            m.setPosition(1,3);
            assertEquals(3,assertInstanceOf(PositionVoltage.class,m.getTalonFX().getAppliedControl()).FeedForward);
            m.setPosition(0);
            var request=assertInstanceOf(PositionVoltage.class,m.getTalonFX().getAppliedControl());
            assertEquals(0,request.Position);assertEquals(0,request.FeedForward);
        } finally {m.getTalonFX().close();CANRegistry.clear();}
    }
    @Test void ordinaryMotionMagicHoldRemovesTrackingFeedforwardEvenAfterStop() {
        assertTrue(HAL.initialize());
        var m=CatalystMotor.builder(33).name("MotionMagicReuse").canBus("can_s0").build();
        try {
            m.setMotionMagicPosition(1,-2);m.stop();m.setMotionMagicPosition(0);
            var request=assertInstanceOf(MotionMagicVoltage.class,m.getTalonFX().getAppliedControl());
            assertEquals(0,request.Position);assertEquals(0,request.FeedForward);
        } finally {m.getTalonFX().close();CANRegistry.clear();}
    }
}
