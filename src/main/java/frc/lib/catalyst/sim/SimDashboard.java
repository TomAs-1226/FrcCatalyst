package frc.lib.catalyst.sim;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.catalyst.mechanisms.CatalystMechanism;
import frc.lib.catalyst.mechanisms.MechanismView;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

/**
 * A generic, mechanism-agnostic simulation dashboard. Register any
 * {@link CatalystMechanism} and the dashboard renders a live, fitting widget for
 * it and lets you drive it from a browser — no per-robot HTML required.
 *
 * <p>Unlike a hand-written cockpit tied to one robot, {@code SimDashboard}
 * discovers each mechanism's shape through {@link CatalystMechanism#describe()}
 * (a {@link MechanismView}). A linear actuator gets a travel bar, a flywheel a
 * speed readout, a claw a state chip, and a team's own subclass gets a widget
 * too — as long as it overrides {@code describe()}. The same dashboard you use
 * in the example runs against your real robot's mechanisms unchanged.
 *
 * <h2>Usage</h2>
 * <pre>{@code
 * private final SimDashboard dash = new SimDashboard();   // port 5805
 *
 * public void robotInit() {
 *     dash.add(elevator)
 *         .slider("Height (m)", 0.0, 0.6, elevator::setSetpoint)
 *         .command("Stow", elevator::stow)
 *         .command("Score", elevator::score);
 *     dash.add(intake)
 *         .command("Run",  intake::intake)
 *         .command("Stop", intake::stop)
 *         .toggle("Game piece", intake::setSimHasPiece);
 *     dash.add(shooter)
 *         .slider("Target (rps)", 0.0, 90.0, shooter::setVelocity);
 *     dash.start();
 * }
 *
 * public void robotPeriodic() {
 *     dash.update();   // drains commands + snapshots state on the main thread
 * }
 * }</pre>
 *
 * <h2>Threading</h2>
 * The HTTP server runs on its own threads. Commands from the browser are
 * <em>never</em> executed there — they are queued and run inside
 * {@link #update()} on the main (scheduler) thread, so it is always safe to
 * schedule Commands or mutate robot state from a control binding. State sent to
 * the browser is also snapshotted in {@link #update()}, so {@code describe()} is
 * only ever called on the main thread.
 *
 * <h2>Real-robot safety</h2>
 * Everything no-ops off simulation. {@link #start()} and {@link #update()} return
 * immediately when {@link RobotBase#isSimulation()} is false, so the same calls
 * can stay in shared robot code without guarding.
 */
public final class SimDashboard {

    /** Default HTTP port (matches the WPILib data-port convention used by the example cockpit). */
    public static final int DEFAULT_PORT = 5805;

    private final int port;
    private final List<Entry> entries = new ArrayList<>();
    private final ConcurrentLinkedQueue<Runnable> commands = new ConcurrentLinkedQueue<>();

    private volatile String stateJson = "{\"mechanisms\":[]}";
    private volatile String layoutJson = "{\"title\":\"SimDashboard\",\"mechanisms\":[]}";
    private String title = "Catalyst SimDashboard";

    private HttpServer server;
    private boolean started;

    public SimDashboard() {
        this(DEFAULT_PORT);
    }

    public SimDashboard(int port) {
        this.port = port;
    }

    /** Override the page title shown in the browser. Returns {@code this} for chaining. */
    public SimDashboard title(String title) {
        this.title = title;
        return this;
    }

    /**
     * Register a mechanism. The returned {@link Handle} lets you attach optional
     * controls (buttons, sliders, toggles). With no controls the mechanism is
     * still shown live — just read-only.
     */
    public Handle add(CatalystMechanism mechanism) {
        Entry e = new Entry(mechanism);
        entries.add(e);
        return new Handle(e);
    }

    /**
     * Start the HTTP server (sim only). Safe to call on a real robot — it does
     * nothing there. Call after every {@link #add} so the layout is complete.
     */
    public void start() {
        if (started) {
            return;
        }
        if (!RobotBase.isSimulation()) {
            return;
        }
        layoutJson = buildLayout();
        stateJson = buildState();
        try {
            server = HttpServer.create(new InetSocketAddress(port), 0);
        } catch (IOException e) {
            throw new RuntimeException("SimDashboard failed to bind port " + port, e);
        }
        server.createContext("/layout", ex -> respond(ex, 200, "application/json", layoutJson));
        server.createContext("/state", ex -> respond(ex, 200, "application/json", stateJson));
        server.createContext("/cmd", this::handleCmd);
        server.createContext("/", ex -> {
            if (!"/".equals(ex.getRequestURI().getPath())) {
                respond(ex, 404, "text/plain", "not found");
                return;
            }
            respond(ex, 200, "text/html", PAGE);
        });
        server.setExecutor(null);
        server.start();
        started = true;
        System.out.println("******** Catalyst SimDashboard: http://localhost:" + port + " ********");
    }

    /**
     * Pump the dashboard: run any queued browser commands and snapshot live
     * state. Call once per loop on the main thread (e.g. in {@code robotPeriodic}
     * or {@code simulationPeriodic}). No-op off simulation / before {@link #start()}.
     */
    public void update() {
        if (server == null) {
            return;
        }
        Runnable cmd;
        while ((cmd = commands.poll()) != null) {
            try {
                cmd.run();
            } catch (RuntimeException ex) {
                System.out.println("[SimDashboard] command error: " + ex);
            }
        }
        stateJson = buildState();
    }

    /** Stop the HTTP server. Safe to call when never started. */
    public void stop() {
        if (server != null) {
            server.stop(0);
            server = null;
            started = false;
        }
    }

    // ------------------------------------------------------------------ control model

    /** Fluent per-mechanism control registration returned by {@link #add}. */
    public final class Handle {
        private final Entry e;

        private Handle(Entry e) {
            this.e = e;
        }

        /** A push button that runs {@code action} on the main thread. */
        public Handle button(String label, Runnable action) {
            e.actions.add(new Action(label, action));
            return this;
        }

        /**
         * A push button that schedules the Command produced by {@code factory}.
         * The factory is invoked on the main thread each press.
         */
        public Handle command(String label, Supplier<Command> factory) {
            return button(label, () -> CommandScheduler.getInstance().schedule(factory.get()));
        }

        /**
         * A continuous slider over {@code [min, max]}. {@code onChange} fires on
         * the main thread whenever the user moves it.
         */
        public Handle slider(String label, double min, double max, DoubleConsumer onChange) {
            e.sliders.add(new Slider(label, min, max, onChange));
            return this;
        }

        /** An on/off switch. {@code onChange} fires on the main thread when flipped. */
        public Handle toggle(String label, Consumer<Boolean> onChange) {
            e.toggles.add(new Toggle(label, onChange, null));
            return this;
        }

        /**
         * An on/off switch that also reflects live state read from {@code state}
         * (so the switch tracks the robot, not just the last click).
         */
        public Handle toggle(String label, Consumer<Boolean> onChange, BooleanSupplier state) {
            e.toggles.add(new Toggle(label, onChange, state));
            return this;
        }

        /** Register another mechanism without going back through the dashboard handle. */
        public Handle add(CatalystMechanism mechanism) {
            return SimDashboard.this.add(mechanism);
        }
    }

    private static final class Action {
        final String label;
        final Runnable run;

        Action(String label, Runnable run) {
            this.label = label;
            this.run = run;
        }
    }

    private static final class Slider {
        final String label;
        final double min;
        final double max;
        final DoubleConsumer onChange;

        Slider(String label, double min, double max, DoubleConsumer onChange) {
            this.label = label;
            this.min = min;
            this.max = max;
            this.onChange = onChange;
        }
    }

    private static final class Toggle {
        final String label;
        final Consumer<Boolean> onChange;
        final BooleanSupplier state;

        Toggle(String label, Consumer<Boolean> onChange, BooleanSupplier state) {
            this.label = label;
            this.onChange = onChange;
            this.state = state;
        }
    }

    private static final class Entry {
        final CatalystMechanism mech;
        final List<Action> actions = new ArrayList<>();
        final List<Slider> sliders = new ArrayList<>();
        final List<Toggle> toggles = new ArrayList<>();

        Entry(CatalystMechanism mech) {
            this.mech = mech;
        }
    }

    // ------------------------------------------------------------------ HTTP plumbing

    private void handleCmd(HttpExchange ex) throws IOException {
        Map<String, String> q = query(ex.getRequestURI());
        try {
            String action = q.get("action");
            if (action != null) {
                int[] ij = pair(action);
                if (ij != null && ij[0] < entries.size()) {
                    Entry e = entries.get(ij[0]);
                    if (ij[1] < e.actions.size()) {
                        Action a = e.actions.get(ij[1]);
                        commands.add(a.run);
                    }
                }
            }
            String slider = q.get("slider");
            if (slider != null) {
                int[] ij = pair(slider);
                double v = parse(q.get("v"));
                if (ij != null && !Double.isNaN(v) && ij[0] < entries.size()) {
                    Entry e = entries.get(ij[0]);
                    if (ij[1] < e.sliders.size()) {
                        Slider s = e.sliders.get(ij[1]);
                        commands.add(() -> s.onChange.accept(v));
                    }
                }
            }
            String toggle = q.get("toggle");
            if (toggle != null) {
                int[] ij = pair(toggle);
                boolean on = !"0".equals(q.get("v"));
                if (ij != null && ij[0] < entries.size()) {
                    Entry e = entries.get(ij[0]);
                    if (ij[1] < e.toggles.size()) {
                        Toggle t = e.toggles.get(ij[1]);
                        commands.add(() -> t.onChange.accept(on));
                    }
                }
            }
        } catch (RuntimeException ignored) {
            // Never let a malformed request take down the HTTP handler.
        }
        respond(ex, 200, "application/json", "{\"ok\":true}");
    }

    /** Parse an {@code "i:j"} index pair, or null if malformed. */
    private static int[] pair(String s) {
        int colon = s.indexOf(':');
        if (colon <= 0 || colon == s.length() - 1) {
            return null;
        }
        try {
            int i = Integer.parseInt(s.substring(0, colon));
            int j = Integer.parseInt(s.substring(colon + 1));
            if (i < 0 || j < 0) {
                return null;
            }
            return new int[] {i, j};
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private static double parse(String s) {
        if (s == null) {
            return Double.NaN;
        }
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return Double.NaN;
        }
    }

    private static void respond(HttpExchange ex, int code, String type, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", type);
        ex.getResponseHeaders().set("Cache-Control", "no-store");
        ex.sendResponseHeaders(code, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }

    private static Map<String, String> query(URI uri) {
        Map<String, String> out = new LinkedHashMap<>();
        String q = uri.getRawQuery();
        if (q == null) {
            return out;
        }
        for (String pair : q.split("&")) {
            int eq = pair.indexOf('=');
            if (eq < 0) {
                out.put(urlDecode(pair), "");
            } else {
                out.put(urlDecode(pair.substring(0, eq)), urlDecode(pair.substring(eq + 1)));
            }
        }
        return out;
    }

    private static String urlDecode(String s) {
        return java.net.URLDecoder.decode(s, StandardCharsets.UTF_8);
    }

    // ------------------------------------------------------------------ JSON building

    private String buildLayout() {
        StringBuilder sb = new StringBuilder();
        sb.append("{\"title\":").append(jsonStr(title)).append(",\"mechanisms\":[");
        for (int i = 0; i < entries.size(); i++) {
            Entry e = entries.get(i);
            if (i > 0) {
                sb.append(',');
            }
            sb.append("{\"name\":").append(jsonStr(e.mech.getMechanismName()));
            sb.append(",\"actions\":[");
            for (int j = 0; j < e.actions.size(); j++) {
                if (j > 0) {
                    sb.append(',');
                }
                sb.append(jsonStr(e.actions.get(j).label));
            }
            sb.append("],\"sliders\":[");
            for (int j = 0; j < e.sliders.size(); j++) {
                Slider s = e.sliders.get(j);
                if (j > 0) {
                    sb.append(',');
                }
                sb.append("{\"label\":").append(jsonStr(s.label))
                  .append(",\"min\":").append(jsonNum(s.min))
                  .append(",\"max\":").append(jsonNum(s.max)).append('}');
            }
            sb.append("],\"toggles\":[");
            for (int j = 0; j < e.toggles.size(); j++) {
                if (j > 0) {
                    sb.append(',');
                }
                sb.append(jsonStr(e.toggles.get(j).label));
            }
            sb.append("]}");
        }
        sb.append("]}");
        return sb.toString();
    }

    private String buildState() {
        StringBuilder sb = new StringBuilder();
        sb.append("{\"mechanisms\":[");
        for (int i = 0; i < entries.size(); i++) {
            Entry e = entries.get(i);
            if (i > 0) {
                sb.append(',');
            }
            MechanismView v = safeDescribe(e.mech);
            sb.append('{');
            sb.append("\"name\":").append(jsonStr(v.name()));
            sb.append(",\"kind\":").append(jsonStr(v.kind()));
            sb.append(",\"value\":").append(jsonNum(v.value()));
            sb.append(",\"unit\":").append(jsonStr(v.unit()));
            sb.append(",\"setpoint\":").append(jsonNum(v.setpoint()));
            sb.append(",\"min\":").append(jsonNum(v.min()));
            sb.append(",\"max\":").append(jsonNum(v.max()));
            sb.append(",\"velocity\":").append(jsonNum(v.velocity()));
            sb.append(",\"current\":").append(jsonNum(v.currentAmps()));
            sb.append(",\"extra\":{");
            int k = 0;
            for (Map.Entry<String, String> ex : v.extra().entrySet()) {
                if (k++ > 0) {
                    sb.append(',');
                }
                sb.append(jsonStr(ex.getKey())).append(':').append(jsonStr(ex.getValue()));
            }
            sb.append('}');
            sb.append(",\"toggles\":[");
            for (int j = 0; j < e.toggles.size(); j++) {
                if (j > 0) {
                    sb.append(',');
                }
                BooleanSupplier st = e.toggles.get(j).state;
                sb.append(st != null && st.getAsBoolean() ? "true" : "false");
            }
            sb.append("]}");
        }
        sb.append("]}");
        return sb.toString();
    }

    private static MechanismView safeDescribe(CatalystMechanism m) {
        try {
            MechanismView v = m.describe();
            return v != null ? v : MechanismView.of(m.getMechanismName(), "generic").build();
        } catch (RuntimeException ex) {
            return MechanismView.of(m.getMechanismName(), "generic")
                    .extra("describeError", ex.getClass().getSimpleName())
                    .build();
        }
    }

    private static String jsonNum(double v) {
        return (Double.isNaN(v) || Double.isInfinite(v)) ? "null" : Double.toString(v);
    }

    private static String jsonStr(String s) {
        if (s == null) {
            return "\"\"";
        }
        StringBuilder sb = new StringBuilder(s.length() + 2);
        sb.append('"');
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
            }
        }
        sb.append('"');
        return sb.toString();
    }

    // ------------------------------------------------------------------ frontend

    private static final String PAGE = """
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Catalyst SimDashboard</title>
<style>
  :root{
    --bg:#0b0f14; --panel:#121a23; --panel2:#0e151d; --line:#22303d;
    --txt:#dfe9f3; --dim:#7d93a6; --accent:#39d0d8; --accent2:#7c5cff;
    --good:#37d67a; --warn:#ffb020; --bad:#ff5470; --fill:#2a3a49;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--txt);
    font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:13px}
  header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:14px;
    padding:12px 18px;background:linear-gradient(180deg,#0e151d,#0b0f14);
    border-bottom:1px solid var(--line)}
  header h1{font-size:15px;margin:0;letter-spacing:.5px}
  header .sub{color:var(--dim);font-size:11px}
  .status{margin-left:auto;display:flex;align-items:center;gap:7px;font-size:11px;color:var(--dim)}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--bad);box-shadow:0 0 8px var(--bad)}
  .dot.live{background:var(--good);box-shadow:0 0 8px var(--good)}
  #grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:14px;padding:18px}
  .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;
    padding:14px 14px 12px;display:flex;flex-direction:column;gap:10px}
  .chead{display:flex;align-items:center;gap:8px}
  .chead .nm{font-weight:600;font-size:14px}
  .badge{font-size:10px;text-transform:uppercase;letter-spacing:.6px;color:#04121a;
    background:var(--accent);border-radius:5px;padding:2px 7px;font-weight:700}
  .readout{display:flex;align-items:baseline;gap:8px}
  .val{font-size:30px;font-weight:700;line-height:1}
  .unit{color:var(--dim);font-size:13px}
  .sp{margin-left:auto;color:var(--dim);font-size:11px;text-align:right}
  .sp b{color:var(--accent2);font-weight:700}
  .bar{position:relative;height:12px;background:var(--panel2);border:1px solid var(--line);
    border-radius:7px;overflow:hidden}
  .bar .fill{position:absolute;top:0;left:0;bottom:0;width:0;
    background:linear-gradient(90deg,var(--accent),var(--accent2));transition:width .08s linear}
  .bar .tick{position:absolute;top:-2px;bottom:-2px;width:2px;background:var(--warn)}
  .rng{display:flex;justify-content:space-between;color:var(--dim);font-size:10px}
  .stats{display:flex;flex-wrap:wrap;gap:6px}
  .chip{background:var(--panel2);border:1px solid var(--line);border-radius:6px;
    padding:3px 8px;font-size:11px;color:var(--dim)}
  .chip b{color:var(--txt);font-weight:600}
  .chip.t{color:#04121a;background:var(--good);border-color:var(--good);font-weight:600}
  .chip.f{color:var(--txt)}
  .controls{display:flex;flex-direction:column;gap:9px;margin-top:2px;
    border-top:1px dashed var(--line);padding-top:10px}
  .btns{display:flex;flex-wrap:wrap;gap:7px}
  button{background:var(--fill);color:var(--txt);border:1px solid var(--line);
    border-radius:7px;padding:6px 11px;font:inherit;font-size:12px;cursor:pointer}
  button:hover{border-color:var(--accent);color:#fff}
  button:active{transform:translateY(1px)}
  .sl{display:flex;align-items:center;gap:8px}
  .sl label{color:var(--dim);font-size:11px;min-width:84px}
  .sl input[type=range]{flex:1;accent-color:var(--accent)}
  .sl .v{min-width:46px;text-align:right;color:var(--accent);font-size:12px}
  .tg{display:flex;align-items:center;gap:8px}
  .tg label{color:var(--dim);font-size:11px}
  .sw{position:relative;width:38px;height:20px;background:var(--panel2);
    border:1px solid var(--line);border-radius:11px;cursor:pointer;transition:.15s}
  .sw.on{background:var(--good);border-color:var(--good)}
  .sw .kn{position:absolute;top:2px;left:2px;width:14px;height:14px;border-radius:50%;
    background:var(--txt);transition:.15s}
  .sw.on .kn{left:20px;background:#04121a}
  .empty{color:var(--dim);padding:40px;text-align:center;grid-column:1/-1}
  .hctrls{margin-left:auto;display:flex;gap:8px;align-items:center}
  .hbtn{background:var(--fill);color:var(--txt);border:1px solid var(--line);border-radius:7px;
    padding:5px 11px;font:inherit;font-size:11px;cursor:pointer}
  .hbtn:hover{border-color:var(--accent);color:#fff}
  .hbtn.on{background:var(--warn);border-color:var(--warn);color:#04121a;font-weight:600}
  .status{margin-left:14px}
  canvas.spark{width:100%;height:34px;display:block;margin:2px 0;
    background:var(--panel2);border:1px solid var(--line);border-radius:6px}
</style>
</head>
<body>
<header>
  <h1>CATALYST <span style="color:var(--accent)">SimDashboard</span></h1>
  <span class="sub" id="sub">generic mechanism cockpit</span>
  <div class="hctrls">
    <button class="hbtn" id="pauseBtn" onclick="togglePause()">Pause</button>
    <button class="hbtn" onclick="exportCsv()">Export CSV</button>
  </div>
  <div class="status"><span id="dot" class="dot"></span><span id="stat">connecting</span></div>
</header>
<div id="grid"><div class="empty">loading mechanisms...</div></div>
<script>
const grid = document.getElementById('grid');
let layout = null;
let live = false;
let paused = false;
let lastState = null;
const HIST = [];          // per-mechanism ring buffer of the primary value
const HCAP = 150;         // ~15s of history at 100ms

function fmt(x){ return (x===null||x===undefined) ? '--' : (Math.round(x*100)/100).toString(); }

function pct(v,min,max){
  if(v===null||min===null||max===null||max<=min) return null;
  let p = (v-min)/(max-min);
  return Math.max(0,Math.min(1,p))*100;
}

function build(){
  if(!layout.mechanisms.length){ grid.innerHTML='<div class="empty">no mechanisms registered</div>'; return; }
  document.getElementById('sub').textContent = layout.mechanisms.length+' mechanisms';
  document.title = layout.title || 'Catalyst SimDashboard';
  grid.innerHTML = layout.mechanisms.map((m,i)=>{
    let ctrl = '';
    if(m.actions.length){
      ctrl += '<div class="btns">'+m.actions.map((a,j)=>
        '<button onclick="act('+i+','+j+')">'+esc(a)+'</button>').join('')+'</div>';
    }
    m.sliders.forEach((s,j)=>{
      const mid = (s.min+s.max)/2;
      ctrl += '<div class="sl"><label>'+esc(s.label)+'</label>'+
        '<input type="range" min="'+s.min+'" max="'+s.max+'" step="'+((s.max-s.min)/200)+'" value="'+mid+'" '+
        'oninput="sld('+i+','+j+',this)"/><span class="v" id="sv'+i+'_'+j+'">'+fmt(mid)+'</span></div>';
    });
    m.toggles.forEach((t,j)=>{
      ctrl += '<div class="tg"><div class="sw" id="tg'+i+'_'+j+'" onclick="tog('+i+','+j+')"><div class="kn"></div></div>'+
        '<label>'+esc(t)+'</label></div>';
    });
    const ctrlBlock = ctrl ? '<div class="controls">'+ctrl+'</div>' : '';
    return '<div class="card" id="card'+i+'">'+
      '<div class="chead"><span class="nm" id="nm'+i+'">'+esc(m.name)+'</span>'+
        '<span class="badge" id="kd'+i+'">--</span></div>'+
      '<div class="readout"><span class="val" id="vv'+i+'">--</span>'+
        '<span class="unit" id="uu'+i+'"></span>'+
        '<span class="sp" id="ss'+i+'"></span></div>'+
      '<div class="bar" id="bw'+i+'"><div class="fill" id="bf'+i+'"></div><div class="tick" id="bt'+i+'"></div></div>'+
      '<div class="rng" id="rg'+i+'"></div>'+
      '<canvas class="spark" id="sp'+i+'" width="320" height="34"></canvas>'+
      '<div class="stats" id="st'+i+'"></div>'+
      ctrlBlock+
    '</div>';
  }).join('');
}

function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

function render(state){
  state.mechanisms.forEach((m,i)=>{
    const kd=document.getElementById('kd'+i); if(!kd) return;
    kd.textContent=m.kind;
    document.getElementById('vv'+i).textContent=fmt(m.value);
    document.getElementById('uu'+i).textContent=m.unit||'';
    const sp=document.getElementById('ss'+i);
    sp.innerHTML = (m.setpoint!==null&&m.setpoint!==undefined) ? ('target <b>'+fmt(m.setpoint)+'</b>') : '';
    const bw=document.getElementById('bw'+i), bf=document.getElementById('bf'+i),
          bt=document.getElementById('bt'+i), rg=document.getElementById('rg'+i);
    const p=pct(m.value,m.min,m.max);
    if(p===null){ bw.style.display='none'; rg.textContent=''; }
    else{
      bw.style.display='block'; bf.style.width=p+'%';
      const tp=pct(m.setpoint,m.min,m.max);
      if(tp===null){ bt.style.display='none'; } else { bt.style.display='block'; bt.style.left=tp+'%'; }
      rg.innerHTML='<span>'+fmt(m.min)+'</span><span>'+fmt(m.max)+'</span>';
    }
    const stats=[];
    if(m.velocity!==null&&m.velocity!==undefined) stats.push('<span class="chip">vel <b>'+fmt(m.velocity)+'</b></span>');
    if(m.current!==null&&m.current!==undefined) stats.push('<span class="chip">amps <b>'+fmt(m.current)+'</b></span>');
    for(const k in m.extra){
      const raw=m.extra[k]; let cls='chip';
      if(raw==='true') cls='chip t'; else if(raw==='false') cls='chip f';
      stats.push('<span class="'+cls+'">'+esc(k)+' <b>'+esc(raw)+'</b></span>');
    }
    document.getElementById('st'+i).innerHTML=stats.join('');
    if(m.toggles){ m.toggles.forEach((on,j)=>{
      const el=document.getElementById('tg'+i+'_'+j); if(el) el.classList.toggle('on',!!on);
    }); }
    if(!HIST[i]) HIST[i]=[];
    if(m.value!==null&&m.value!==undefined&&isFinite(m.value)){
      HIST[i].push(m.value); if(HIST[i].length>HCAP) HIST[i].shift();
      drawSpark(i, m.setpoint);
    }
  });
}

function drawSpark(i, setpoint){
  const cv=document.getElementById('sp'+i); if(!cv) return;
  const h=HIST[i]; const ctx=cv.getContext('2d'); const W=cv.width, H=cv.height;
  ctx.clearRect(0,0,W,H);
  if(h.length<2) return;
  let lo=Math.min.apply(null,h), hi=Math.max.apply(null,h);
  if(setpoint!==null&&setpoint!==undefined&&isFinite(setpoint)){ lo=Math.min(lo,setpoint); hi=Math.max(hi,setpoint); }
  if(hi-lo<1e-9){ hi=lo+1; lo=lo-1; }
  const pad=2, span=hi-lo;
  const x=k=> pad + k*(W-2*pad)/(HCAP-1);
  const y=v=> H-pad - (v-lo)/span*(H-2*pad);
  // setpoint guide
  if(setpoint!==null&&setpoint!==undefined&&isFinite(setpoint)){
    ctx.strokeStyle='rgba(255,176,32,0.5)'; ctx.lineWidth=1; ctx.beginPath();
    ctx.moveTo(0,y(setpoint)); ctx.lineTo(W,y(setpoint)); ctx.stroke();
  }
  // value trace
  const start=HCAP-h.length;
  ctx.strokeStyle='#39d0d8'; ctx.lineWidth=1.5; ctx.beginPath();
  for(let k=0;k<h.length;k++){ const px=x(start+k), py=y(h[k]); if(k===0) ctx.moveTo(px,py); else ctx.lineTo(px,py); }
  ctx.stroke();
}

function togglePause(){
  paused=!paused;
  const b=document.getElementById('pauseBtn');
  b.textContent=paused?'Resume':'Pause'; b.classList.toggle('on',paused);
}

function exportCsv(){
  if(!lastState) return;
  const nl=String.fromCharCode(10);
  const rows=[['name','kind','value','unit','setpoint','min','max','velocity','current']];
  lastState.mechanisms.forEach(m=>rows.push([m.name,m.kind,m.value,m.unit,m.setpoint,m.min,m.max,m.velocity,m.current]));
  // RFC-4180 quote every field so a comma/newline/quote in a mechanism name
  // can't corrupt the CSV; guard the =,+,-,@ formula-injection prefixes too.
  const q=c=>{ let s=(c===null||c===undefined)?'':String(c); if(/^[-=+@]/.test(s)) s="'"+s; return '"'+s.replace(/"/g,'""')+'"'; };
  const csv=rows.map(r=>r.map(q).join(',')).join(nl);
  const blob=new Blob([csv],{type:'text/csv'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(blob); a.download='simdashboard.csv'; a.click();
  setTimeout(()=>URL.revokeObjectURL(a.href),1000);
}

function setLive(ok){
  live=ok;
  document.getElementById('dot').classList.toggle('live',ok);
  document.getElementById('stat').textContent= ok ? 'live' : 'connecting';
}

function act(i,j){ fetch('/cmd?action='+i+':'+j); }
function sld(i,j,el){ document.getElementById('sv'+i+'_'+j).textContent=fmt(parseFloat(el.value));
  fetch('/cmd?slider='+i+':'+j+'&v='+encodeURIComponent(el.value)); }
function tog(i,j){ const el=document.getElementById('tg'+i+'_'+j); const on=!el.classList.contains('on');
  el.classList.toggle('on',on); fetch('/cmd?toggle='+i+':'+j+'&v='+(on?1:0)); }

async function poll(){
  if(!paused){
    try{
      const r=await fetch('/state',{cache:'no-store'});
      const s=await r.json();
      lastState=s; setLive(true); render(s);
    }catch(e){ setLive(false); }
  }
  setTimeout(poll,100);
}

async function init(){
  try{
    layout=await (await fetch('/layout',{cache:'no-store'})).json();
    build(); poll();
  }catch(e){ grid.innerHTML='<div class="empty">failed to load layout</div>'; setTimeout(init,1000); }
}
init();
</script>
</body>
</html>
""";
}
