package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** The motor history file goes through this both ways, so both ways are tested. */
class MiniJsonTest {

    @Test
    void aDocumentSurvivesTheRoundTrip() {
        Map<String, Object> doc = new LinkedHashMap<>();
        doc.put("name", "FL_Drive \"quoted\" \\ back\nslash");
        doc.put("id", 30);
        doc.put("ratio", 6.75);
        doc.put("big", 1234567890123L);
        doc.put("on", true);
        doc.put("nothing", null);
        doc.put("list", List.of(1, 2.5, "three", false));
        Map<String, Object> nested = new LinkedHashMap<>();
        nested.put("x", -0.5);
        doc.put("nested", nested);

        String text = MiniJson.pretty(doc);
        Map<String, Object> back = MiniJson.readObject(text);

        assertEquals("FL_Drive \"quoted\" \\ back\nslash", back.get("name"));
        assertEquals(30.0, back.get("id"));
        assertEquals(6.75, back.get("ratio"));
        assertEquals(1234567890123.0, back.get("big"));
        assertEquals(Boolean.TRUE, back.get("on"));
        assertNull(back.get("nothing"));
        assertTrue(back.containsKey("nothing"), "an explicit null is a key with a null value");
        assertEquals(List.of(1.0, 2.5, "three", false), back.get("list"));
        assertEquals(-0.5, ((Map<?, ?>) back.get("nested")).get("x"));
        assertEquals(List.of("name", "id", "ratio", "big", "on", "nothing", "list", "nested"),
                List.copyOf(back.keySet()), "key order is kept, so the file reads the way it was written");
    }

    @Test
    void integersAreWrittenWithoutADecimalPoint() {
        Map<String, Object> m = new LinkedHashMap<>();
        m.put("a", 3.0);
        m.put("b", 2.5);
        m.put("c", new long[] {1, 2});
        assertEquals("{\"a\":3,\"b\":2.5,\"c\":[1,2]}", MiniJson.write(m));
    }

    @Test
    void nanBecomesNullRatherThanInvalidJson() {
        String text = MiniJson.write(Map.of("t", Double.NaN));
        assertEquals("{\"t\":null}", text);
        assertNull(MiniJson.readObject(text).get("t"));
    }

    @Test
    void whatIsNotJsonIsRefusedWithAnOffset() {
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> MiniJson.read("{\"a\": tru}"));
        assertTrue(e.getMessage().contains("offset"), e.getMessage());
        assertThrows(IllegalArgumentException.class, () -> MiniJson.read("[1, 2"));
        assertThrows(IllegalArgumentException.class, () -> MiniJson.read("{} trailing"));
        assertThrows(IllegalArgumentException.class, () -> MiniJson.readObject("[1]"));
    }

    @Test
    void thePhoenixDeviceListParses() {
        // Verbatim shape from a Systemcore's diagnostic server, including the escaped nothing.
        String body = "{\"BusUtilPerc\": -1.0, \"DeviceArray\": [{\"BootloaderRev\": \"0.5\", \"CANbus\": \"can_s2\","
                + " \"ID\": 24, \"IsPROLicensed\": false, \"LicenseSigs\": [\"40b5\"], \"Model\": \"Talon FX\","
                + " \"Name\": \"FL_Steer\", \"SerialNo\": \"000E0B500C776800000A00011A0000F4\","
                + " \"CurrentVers\": \"26.1.1.1 (Phoenix 6)\"}], \"GeneralReturn\": {\"Error\": 0}}";
        Map<String, Object> doc = MiniJson.readObject(body);
        List<?> arr = (List<?>) doc.get("DeviceArray");
        Map<?, ?> dev = (Map<?, ?>) arr.get(0);
        assertEquals("FL_Steer", dev.get("Name"));
        assertEquals(24.0, dev.get("ID"));
        assertEquals(-1.0, doc.get("BusUtilPerc"));
    }
}
