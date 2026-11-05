package frc.lib.catalyst.util;
import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import frc.lib.catalyst.hardware.CatalystMotor;
import frc.lib.catalyst.hardware.CANRegistry;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.networktables.NetworkTableInstance;

class TunableGainsFanoutTest {
    /**
     * Read one config group, retrying until a deadline. Under the full suite a simulated config read
     * can miss a one-second timeout (seen 2026-11-01, 1 of 1109 tests); this checks values, not latency.
     */
    private static <T extends com.ctre.phoenix6.configs.ParentConfiguration> T read(CatalystMotor m, T group) {
        com.ctre.phoenix6.StatusCode last=null;
        double deadline=System.nanoTime()/1e9+5.0;
        do {
            last=group instanceof Slot0Configs s?m.getTalonFX().getConfigurator().refresh(s,1)
                :m.getTalonFX().getConfigurator().refresh((MotionMagicConfigs)group,1);
            if(last.isOK())return group;
        } while(System.nanoTime()/1e9<deadline);
        return fail("id "+m.getTalonFX().getDeviceID()+" config unread: "+last);
    }

    @Test void oneDashboardEditUpdatesBothIndependentMotorsAndMotionProfiles() {
        assertTrue(HAL.initialize());boolean enabled=TunableNumber.isTuningEnabled();TunableNumber.enableTuning();
        var a=CatalystMotor.builder(34).name("GainsA").canBus("can_s0").pid(1,0,0).motionMagic(2,3,4).build();
        var b=CatalystMotor.builder(35).name("GainsB").canBus("can_s0").pid(1,0,0).motionMagic(2,3,4).build();
        try {
            var gains=new TunableGains("AuditPair",1,0,0,0,0,0,0,2,3,4);
            var table=NetworkTableInstance.getDefault().getTable("Catalyst/Tuning");
            table.getEntry("AuditPair/kP").setDouble(7);
            table.getEntry("AuditPair/kV").setDouble(.2);
            table.getEntry("AuditPair/MM/Acceleration").setDouble(8);
            gains.checkAndApply(a,b);
            for(var m:new CatalystMotor[]{a,b}) {
                var slot=read(m,new Slot0Configs());
                assertEquals(7,slot.kP,1e-9,"both independent flywheel motors need the same live gains");
                assertEquals((double)(float).2,slot.kV,1e-9); // Phoenix config round-trips this field as float32.
                var mm=read(m,new MotionMagicConfigs());
                assertEquals(8,mm.MotionMagicAcceleration,1e-9);
            }
            // A second edit must propagate too, rather than relying on a boot-only write.
            table.getEntry("AuditPair/kP").setDouble(9);gains.checkAndApply(a,b);
            for(var m:new CatalystMotor[]{a,b}) {
                var slot=read(m,new Slot0Configs());
                assertEquals(9,slot.kP,1e-9);
            }
        } finally {a.getTalonFX().close();b.getTalonFX().close();CANRegistry.clear();
            if(!enabled)TunableNumber.disableTuning();}
    }
}
