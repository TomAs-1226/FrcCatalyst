package frc.robot;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URI;
import java.nio.charset.StandardCharsets;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A tiny, dependency-free web dashboard for the simulator — the "Sim Cockpit."
 * Runs <b>only in simulation</b> and serves a top-down field view of the robot
 * at <a href="http://localhost:5805">http://localhost:5805</a> so you can
 * actually <em>see</em> the turret track the goal, the flywheel spin up, fuel
 * get shot, read errors, and drive the robot from the browser (WASD / arrows).
 *
 * <p>It uses the JDK's built-in {@link HttpServer} (no extra dependencies). This
 * class lives in the example, not the library — it's a demonstration harness.
 */
public final class SimCockpit {

    /** Field-relative drive request from the dashboard: vx, vy (m/s) and yaw rate (rad/s). */
    @FunctionalInterface
    public interface DriveInput {
        void accept(double vx, double vy, double omega);
    }

    private final HttpServer server;

    public SimCockpit(int port,
                      Supplier<String> stateJson,
                      Consumer<String> goalRequest,
                      Consumer<Boolean> enableRequest,
                      DriveInput driveInput,
                      Runnable resetStats,
                      Runnable runAuto,
                      Consumer<String> faultRequest,
                      Consumer<String> toggleRequest) {
        try {
            server = HttpServer.create(new InetSocketAddress(port), 0);
        } catch (IOException e) {
            throw new RuntimeException("SimCockpit failed to bind port " + port, e);
        }

        server.createContext("/state", ex -> respond(ex, 200, "application/json", stateJson.get()));
        // The same heightmap the simulation collides against. Serving it means the map you look at
        // and the geometry you hit are one thing rather than two drawings that drift apart.
        server.createContext("/collision", ex -> {
            // Deploy resolves differently under `simulateJava` than on the roboRIO, and a silent 404
            // just makes the map quietly wrong. Look in the places it can be, and say which failed.
            java.nio.file.Path[] candidates = {
                edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath()
                        .resolve("field-collision.json"),
                java.nio.file.Path.of("src", "main", "deploy", "field-collision.json"),
                java.nio.file.Path.of("example", "src", "main", "deploy", "field-collision.json"),
            };
            for (java.nio.file.Path p : candidates) {
                if (java.nio.file.Files.exists(p)) {
                    try {
                        respond(ex, 200, "application/json", java.nio.file.Files.readString(p));
                        return;
                    } catch (Exception e) {
                        // fall through and try the next one
                    }
                }
            }
            System.err.println("[cockpit] no field-collision.json; looked in "
                    + java.util.Arrays.toString(candidates));
            respond(ex, 404, "application/json", "{}");
        });

        server.createContext("/cmd", ex -> {
            var p = query(ex.getRequestURI());
            if (p.containsKey("goal")) goalRequest.accept(p.get("goal"));
            if (p.containsKey("enabled")) enableRequest.accept(Boolean.parseBoolean(p.get("enabled")));
            if (p.containsKey("vx") || p.containsKey("vy") || p.containsKey("w")) {
                driveInput.accept(parse(p.get("vx")), parse(p.get("vy")), parse(p.get("w")));
            }
            if (p.containsKey("reset")) {
                resetStats.run();
            }
            if (p.containsKey("auto")) {
                runAuto.run();
            }
            if (p.containsKey("fault")) {
                faultRequest.accept(p.get("fault"));
            }
            if (p.containsKey("toggle")) {
                toggleRequest.accept(p.get("toggle"));
            }
            respond(ex, 200, "application/json", "{\"ok\":true}");
        });

        server.createContext("/", ex -> {
            if (!"/".equals(ex.getRequestURI().getPath())) {
                respond(ex, 404, "text/plain", "not found");
                return;
            }
            respond(ex, 200, "text/html", PAGE);
        });

        server.setExecutor(null);
        server.start();
        System.out.println("******** Catalyst Sim Cockpit: http://localhost:" + port + " ********");
    }

    public void stop() {
        server.stop(0);
    }

    private static double parse(String s) {
        try { return s == null ? 0 : Double.parseDouble(s); } catch (NumberFormatException e) { return 0; }
    }

    private static void respond(HttpExchange ex, int code, String type, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", type);
        ex.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
        ex.sendResponseHeaders(code, bytes.length);
        try (OutputStream os = ex.getResponseBody()) {
            os.write(bytes);
        }
    }

    private static java.util.Map<String, String> query(URI uri) {
        var map = new java.util.HashMap<String, String>();
        String q = uri.getRawQuery();
        if (q == null) return map;
        for (String pair : q.split("&")) {
            int i = pair.indexOf('=');
            if (i > 0) {
                map.put(java.net.URLDecoder.decode(pair.substring(0, i), StandardCharsets.UTF_8),
                        java.net.URLDecoder.decode(pair.substring(i + 1), StandardCharsets.UTF_8));
            }
        }
        return map;
    }

    private static final String PAGE = """
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Catalyst Sim Cockpit</title>
<style>
  :root{--bg:#0f1424;--panel:#1b2240;--panel2:#131831;--accent:#e94560;--accent2:#f0648a;
        --text:#f4f4f8;--muted:#8a8aa8;--border:rgba(255,255,255,.08);--ok:#4ade80;--warn:#fbbf24;--err:#f87171;--blue:#60a5fa;--fuel:#f9a825;}
  *{box-sizing:border-box}
  body{margin:0;background:radial-gradient(ellipse at 100% 0%,rgba(96,165,250,.10),transparent 60%),
       radial-gradient(ellipse at 0% 100%,rgba(233,69,96,.08),transparent 60%),var(--bg);
       color:var(--text);font-family:ui-sans-serif,system-ui,-apple-system,"SF Pro Text",Segoe UI,sans-serif;min-height:100vh}
  header{padding:13px 24px;border-bottom:1px solid var(--border);display:flex;align-items:center;gap:14px;
         position:sticky;top:0;background:linear-gradient(180deg,rgba(27,34,64,.92),rgba(27,34,64,.6));backdrop-filter:blur(6px);z-index:5}
  header h1{margin:0;font-size:17px;font-weight:800}
  .badge{padding:2px 10px;border-radius:999px;background:var(--accent);color:#fff;font-size:11px;font-weight:700}
  .conn{margin-left:auto;font-size:12px;color:var(--muted);display:flex;align-items:center;gap:7px}
  .dot{width:9px;height:9px;border-radius:50%;background:var(--err);box-shadow:0 0 8px currentColor}
  .dot.live{background:var(--ok)}
  main{display:grid;grid-template-columns:1fr 340px;gap:18px;padding:18px 22px}
  .card{background:linear-gradient(180deg,var(--panel),var(--panel2));border:1px solid var(--border);border-radius:14px;
        padding:15px 17px;box-shadow:0 8px 24px rgba(0,0,0,.25)}
  .card h2{margin:0 0 11px;font-size:11px;text-transform:uppercase;letter-spacing:1.3px;color:var(--muted);font-weight:800}
  canvas{width:100%;display:block;border-radius:10px;background:#0a1120}
  .hint{margin-top:9px;font-size:12px;color:var(--muted)}
  .hint b{color:var(--accent2)}
  .row{display:flex;justify-content:space-between;padding:5px 0;font-size:13px;border-bottom:1px dashed var(--border)}
  .row:last-child{border-bottom:0}
  .row .v{font-family:ui-monospace,Menlo,monospace;color:var(--accent2);font-weight:700}
  .pill{display:inline-block;padding:2px 9px;border-radius:999px;font-size:11px;font-weight:800}
  .pill.ok{background:rgba(74,222,128,.16);color:var(--ok)} .pill.no{background:rgba(248,113,113,.16);color:var(--err)}
  .controls{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:4px}
  button{background:rgba(255,255,255,.04);color:var(--text);border:1px solid var(--border);border-radius:9px;
         padding:11px;font-size:13px;font-weight:700;cursor:pointer;transition:.12s}
  button:hover{transform:translateY(-1px);border-color:var(--accent)}
  button.active{background:linear-gradient(135deg,var(--accent),#ff6b8a);border-color:transparent;box-shadow:0 4px 12px rgba(233,69,96,.35)}
  button.en{grid-column:span 2}
  button.en.on{background:linear-gradient(135deg,#16a34a,#4ade80);border-color:transparent;color:#04210f}
  .bar{height:11px;border-radius:999px;background:rgba(255,255,255,.07);overflow:hidden;margin-top:5px}
  .bar>i{display:block;height:100%;background:linear-gradient(90deg,#f9a825,#ffd166);width:0;transition:width .1s;box-shadow:0 0 8px rgba(249,168,37,.5)}
  .log{max-height:120px;overflow:auto;font-family:ui-monospace,Menlo,monospace;font-size:12px;line-height:1.6}
  .log .e{color:var(--err)} .log .w{color:var(--warn)} .log .empty{color:var(--muted)}
  .code{margin:0;font-family:ui-monospace,"SF Mono",Menlo,monospace;font-size:12px;line-height:1.5;color:#7e88ad}
  .code .ln{display:block;padding:1px 7px;border-radius:5px;border-left:2px solid transparent;transition:background .12s}
  .code .live{color:#eef1ff;background:rgba(124,58,237,.16);border-left-color:#a78bfa}
  .code .kw{color:#f0648a} .code .fn{color:#60a5fa} .code .cm{color:#566087}
  .code.math{color:#9aa3c4} .code.math .num{color:#4ade80;font-weight:700}
  .code .dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:#a78bfa;margin-right:7px;
             box-shadow:0 0 7px #a78bfa;vertical-align:middle;opacity:0}
  .code .live .dot{opacity:1}
  .h2sub{color:var(--muted);font-weight:600;text-transform:none;letter-spacing:0}
  .chartlbl{font-size:11px;color:var(--muted);margin-top:3px;display:flex;gap:12px}
  canvas#chartRpm,canvas#chartErr{height:auto}
  @media(max-width:1080px){main{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <h1>Catalyst <span style="color:var(--accent2)">Sim Cockpit</span></h1>
  <span class="badge" id="mode">DISABLED</span>
  <span class="badge" id="clock" style="background:#27305a;font-family:ui-monospace,monospace">2:15</span>
  <span class="conn"><span class="dot" id="dot"></span><span id="connTxt">connecting…</span></span>
</header>
<main>
  <div style="display:flex;flex-direction:column;gap:16px;min-width:0">
    <section class="card">
      <h2>REBUILT 2026 <span class="h2sub">· top-down field</span></h2>
      <!-- 840x416 is 2.019:1, not the field's 2.050:1, which drew everything 1.5% too tall and a
           square robot as a rectangle. The height below is the field aspect; sizeView() redoes it
           from the map's own header once /collision answers. -->
      <canvas id="view" width="840" height="410"></canvas>
      <div class="hint"><b>WASD</b> / arrows drive (field-oriented) · <b>Q / E</b> rotate. The turret holds the hub and
         leads the <b>virtual goal</b> (Shoot-On-The-Fly). Solid obstacles · you only score from the blue zone.</div>
    </section>
    <section class="card">
      <h2>Catalyst code <span class="h2sub">· live — what's executing this loop</span></h2>
      <pre class="code" id="code"></pre>
    </section>
    <section class="card">
      <h2>Math core <span class="h2sub">· AimingSolver.solve() with live values</span></h2>
      <pre class="code math" id="math"></pre>
    </section>
  </div>
  <div style="display:flex;flex-direction:column;gap:16px">
    <section class="card">
      <h2>Intent</h2>
      <div class="row"><span>Phase</span><span id="phase" class="pill" style="background:#27305a;color:#cdd6ff">—</span></div>
      <div class="row"><span>Blue Hub</span><span id="hub" class="pill no">INACTIVE</span></div>
      <div class="row"><span>Fuel scored</span><span class="v" id="scored" style="color:var(--ok)">0</span></div>
      <div class="row"><span>Active goal</span><span class="v" id="gActive">—</span></div>
      <div class="row"><span>Ready to shoot</span><span id="gReady" class="pill no">NO</span></div>
      <div class="row"><span>Fuel onboard</span><span class="v" id="fuelTxt">0 / 50</span></div>
      <div class="bar"><i id="fuelBar"></i></div>
      <div class="row" style="margin-top:8px"><span>Shots fired</span><span class="v" id="shots">0</span></div>
      <div class="row"><span>Hits / Misses</span><span class="v" id="hm">0 / 0</span></div>
      <div class="row"><span>Accuracy (actual)</span><span class="v" id="acc">—</span></div>
      <div class="row"><span>Accuracy (ideal math)</span><span class="v" id="iacc" style="color:var(--ok)">—</span></div>
    </section>
    <section class="card">
      <h2>Aiming &amp; Shooter</h2>
      <div class="row"><span>Distance to goal</span><span class="v" id="dist">—</span></div>
      <div class="row"><span>Shot time (TOF)</span><span class="v" id="tof">—</span></div>
      <div class="row"><span>Turret bearing</span><span class="v" id="tb">—</span></div>
      <div class="row"><span>Flywheel RPM</span><span class="v" id="rpm">—</span></div>
      <div class="row"><span>At speed</span><span id="atSpeed" class="pill no">NO</span></div>
      <div class="row"><span>Hood</span><span class="v" id="hood">—</span></div>
    </section>
    <section class="card">
      <h2>Telemetry <span class="h2sub">· live control traces</span></h2>
      <canvas id="chartRpm" width="320" height="76"></canvas>
      <div class="chartlbl"><span style="color:#4ade80">● flywheel RPM</span><span>target (grey)</span></div>
      <canvas id="chartErr" width="320" height="64" style="margin-top:10px"></canvas>
      <div class="chartlbl"><span style="color:#60a5fa">● turret aim error</span><span>°, lower = tighter</span></div>
    </section>
    <section class="card">
      <h2>System safety <span class="h2sub">· RobotSafety + AlertManager</span></h2>
      <div class="row"><span>RobotSafety</span><span id="safety" class="pill ok">OK</span></div>
      <div class="row" id="safetyReasonRow" style="display:none"><span>Reason</span><span class="v" id="safetyReason" style="font-size:11px">—</span></div>
      <div class="controls" style="margin-top:6px">
        <button onclick="fault('overheat')">⚠ Turret overheat</button>
        <button onclick="fault('stall')">⚠ Shooter stall</button>
        <button onclick="fault('brownout')">⚠ Brownout</button>
        <button onclick="fault('clear')" style="background:linear-gradient(135deg,#16a34a,#4ade80);border-color:transparent;color:#04210f">✓ Clear faults</button>
      </div>
    </section>
    <section class="card">
      <h2>Drive the robot</h2>
      <div class="controls">
        <button class="en" id="enBtn" onclick="toggleEnable()">● ENABLE</button>
        <button data-goal="Idle" onclick="goal('Idle')">Idle</button>
        <button data-goal="Intake" onclick="goal('Intake')">Intake</button>
        <button data-goal="AimShoot" onclick="goal('AimShoot')" style="grid-column:span 2">Aim &amp; Shoot</button>
        <button onclick="runAuto()" style="grid-column:span 2;background:linear-gradient(135deg,#7c3aed,#a78bfa);border-color:transparent;color:#fff">▶ Run 20s Autonomous</button>
        <button onclick="resetStats()" style="grid-column:span 2">↺ Reset stats &amp; clock</button>
      </div>
    </section>
    <section class="card">
      <h2>Sim features <span class="h2sub">· toggle on/off</span></h2>
      <div class="controls">
        <button id="tCopilot" onclick="toggle('copilot')">🤖 Co-Pilot</button>
        <button id="tClimb" onclick="toggle('climb')">🪜 Climb</button>
        <button id="tOpp" onclick="toggle('opponent')">🛡 Opponent</button>
        <button id="tGhost" onclick="toggle('ghost')">👻 Ghost replay</button>
      </div>
      <div id="copilotBox" style="display:none;margin-top:11px">
        <div class="row"><span>Strategist · Collect</span><span class="v" id="scCollect">0</span></div>
        <div class="bar"><i id="barCollect" style="background:linear-gradient(90deg,#f9a825,#ffd166)"></i></div>
        <div class="row" style="margin-top:5px"><span>Strategist · Score</span><span class="v" id="scScore">0</span></div>
        <div class="bar"><i id="barScore" style="background:linear-gradient(90deg,#4ade80,#86efac)"></i></div>
        <div class="row" style="margin-top:5px"><span>Winner (running)</span><span id="copilotWin" class="pill no">—</span></div>
      </div>
      <div id="climbBox" style="display:none;margin-top:11px">
        <div class="row"><span>Climb height</span><span class="v" id="climbPct">0%</span></div>
        <div class="bar"><i id="barClimb" style="background:linear-gradient(90deg,#a78bfa,#c4b5fd)"></i></div>
      </div>
      <div class="hint" id="ghostHint" style="display:none">Run a 20s auto first to record the path, then toggle Ghost to replay it (purple).</div>
    </section>
    <section class="card">
      <h2>Alerts &amp; Errors</h2>
      <div class="log" id="log"><span class="empty">No alerts.</span></div>
    </section>
  </div>
</main>
<script>
const $=id=>document.getElementById(id);
let last=null, enabled=false, lastShotId=0, balls=[], splashes=[];
const keys={};

function goal(n){ fetch('/cmd?goal='+encodeURIComponent(n)); }
function toggleEnable(){ fetch('/cmd?enabled='+(!enabled)); }
function resetStats(){ fetch('/cmd?reset=1'); }
function runAuto(){ fetch('/cmd?auto=1'); }
function fault(n){ fetch('/cmd?fault='+n); }
function toggle(n){ fetch('/cmd?toggle='+n); }

// rolling telemetry buffers for the live strip charts
const NB=90, buf={rpm:[],rpmT:[],err:[]};
function pushTelem(s){
  buf.rpm.push(s.shooter.rpm); buf.rpmT.push(s.shooter.target);
  buf.err.push(Math.abs(((s.turret.field - s.turret.aimField + 540)%360)-180));
  for(const k in buf){ if(buf[k].length>NB) buf[k].shift(); }
}
function lineChart(cid,series,maxY,colors){
  const c=$(cid); if(!c) return; const x=c.getContext('2d'),W=c.width,H=c.height;
  x.clearRect(0,0,W,H);
  x.strokeStyle='rgba(255,255,255,0.05)'; x.lineWidth=1;
  for(let i=1;i<3;i++){x.beginPath();x.moveTo(0,H*i/3);x.lineTo(W,H*i/3);x.stroke();}
  series.forEach((data,si)=>{ if(!data.length) return;
    x.strokeStyle=colors[si]; x.lineWidth=1.8; x.beginPath();
    data.forEach((v,i)=>{ const px=i/(NB-1)*W, py=H-Math.min(1,Math.max(0,v)/maxY)*(H-4)-2; i?x.lineTo(px,py):x.moveTo(px,py); });
    x.stroke(); });
}
function drawCharts(){
  lineChart('chartRpm',[buf.rpmT,buf.rpm],6000,['rgba(138,138,168,0.55)','#4ade80']);
  lineChart('chartErr',[buf.err],20,['#60a5fa']);
}

// --- keyboard driving (field frame: +x right, +y up; Q/E rotate) ---
let drvX=0, drvY=0, drvW=0;
const SPEED=4.6, ROT=2.2;
const DRIVEKEYS=['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright'];
function updateDrive(){
  let vx=0, vy=0, w=0;
  if(keys['w']||keys['arrowup']) vy+=1;
  if(keys['s']||keys['arrowdown']) vy-=1;
  if(keys['d']||keys['arrowright']) vx+=1;
  if(keys['a']||keys['arrowleft']) vx-=1;
  if(keys['q']) w+=1;   // CCW
  if(keys['e']) w-=1;   // CW
  const nx=vx*SPEED, ny=vy*SPEED, nw=w*ROT;
  if(nx!==drvX||ny!==drvY||nw!==drvW){ drvX=nx; drvY=ny; drvW=nw; fetch('/cmd?vx='+nx+'&vy='+ny+'&w='+nw); }
}
addEventListener('keydown',e=>{const k=e.key.toLowerCase(); if(DRIVEKEYS.includes(k)){keys[k]=true;updateDrive();e.preventDefault();}});
addEventListener('keyup',e=>{const k=e.key.toLowerCase(); if(keys[k]!==undefined){keys[k]=false;updateDrive();e.preventDefault();}});

async function poll(){
  try{ const r=await fetch('/state',{cache:'no-store'}); last=await r.json(); render(last);
       $('dot').classList.add('live'); $('connTxt').textContent='connected';
  }catch(e){ $('dot').classList.remove('live'); $('connTxt').textContent='waiting for robot…'; }
}

function render(s){
  enabled=s.enabled;
  $('mode').textContent = s.enabled ? 'TELEOP' : 'DISABLED';
  $('mode').style.background = s.enabled ? 'var(--ok)' : 'var(--muted)';
  $('mode').style.color = s.enabled ? '#04210f' : '#fff';
  const eb=$('enBtn'); eb.classList.toggle('on',s.enabled); eb.textContent=(s.enabled?'● ENABLED':'● ENABLE');
  const mt=s.matchTime||0, cl=$('clock');
  cl.textContent=Math.floor(mt/60)+':'+String(Math.floor(mt%60)).padStart(2,'0');
  cl.style.background = mt<=30 ? 'var(--err)' : (mt<=0.01?'var(--muted)':'#27305a');

  const ph=$('phase'), m=s.match||{phase:'TELEOP',hubActive:false};
  ph.textContent=m.phase + (s.auto&&s.auto.running?' (running plan)':'');
  ph.style.background = m.phase==='AUTO' ? 'rgba(124,58,237,.3)' : '#27305a';
  const hb=$('hub'); hb.textContent=m.hubActive?'ACTIVE':'INACTIVE'; hb.className='pill '+(m.hubActive?'ok':'no');
  $('scored').textContent=(s.scored||0)+(s.auto&&s.auto.running?(' (auto '+s.auto.scored+')'):'');
  $('gActive').textContent=s.goal.active;
  const gr=$('gReady'); gr.textContent=s.goal.ready?'YES':'NO'; gr.className='pill '+(s.goal.ready?'ok':'no');
  $('fuelTxt').textContent=s.fuel+' / 50';
  $('fuelBar').style.width=(Math.min(s.fuel,50)/50*100)+'%';
  $('shots').textContent=s.shots;
  $('hm').textContent=s.hits+' / '+s.misses;
  $('acc').textContent=s.shots>0?Math.round(s.hits/s.shots*100)+'%':'—';
  $('iacc').textContent=s.shots>0?Math.round((s.idealHits||0)/s.shots*100)+'%':'—';

  $('dist').textContent=s.aim.distance.toFixed(2)+' m';
  $('tof').textContent=(s.aim.shotTime*1000).toFixed(0)+' ms';
  $('tb').textContent=s.turret.field.toFixed(0)+'\\u00B0';
  $('rpm').textContent=s.shooter.rpm.toFixed(0)+' / '+s.shooter.target.toFixed(0);
  const as=$('atSpeed'); as.textContent=s.shooter.atSpeed?'YES':'NO'; as.className='pill '+(s.shooter.atSpeed?'ok':'no');
  $('hood').textContent=s.hood.deg.toFixed(0)+'\\u00B0 / '+s.hood.target.toFixed(0)+'\\u00B0';

  document.querySelectorAll('button[data-goal]').forEach(b=>b.classList.toggle('active',b.dataset.goal===s.goal.active));

  const log=$('log'); const items=[];
  (s.errors||[]).forEach(e=>items.push('<div class="e">\\u26d4 '+esc(e)+'</div>'));
  (s.warnings||[]).forEach(w=>items.push('<div class="w">\\u26a0 '+esc(w)+'</div>'));
  log.innerHTML=items.length?items.join(''):'<span class="empty">No alerts.</span>';

  // spawn a ball for each new shot event (carries its own landing + hit flag)
  (s.recentShots||[]).forEach(r=>{ if(r.id>lastShotId){ spawnShot(r); lastShotId=r.id; } });
  renderCode(s);
  renderMath(s);
  // safety status
  const sf=s.safety||{tripped:false,reason:''};
  const sfp=$('safety'); sfp.textContent=sf.tripped?'TRIPPED':'OK'; sfp.className='pill '+(sf.tripped?'no':'ok');
  $('safetyReasonRow').style.display=sf.tripped?'flex':'none'; $('safetyReason').textContent=sf.reason||'—';
  // telemetry strip charts
  pushTelem(s); drawCharts();
  // sim feature toggles + readouts
  const cp=s.copilot||{}, clb=s.climb||{}, op=s.opp||{}, gh=s.ghost||{};
  $('tCopilot').classList.toggle('active',!!cp.on);
  $('tClimb').classList.toggle('active',!!clb.on);
  $('tOpp').classList.toggle('active',!!op.on);
  $('tGhost').classList.toggle('active',!!gh.on);
  $('copilotBox').style.display=cp.on?'block':'none';
  if(cp.on){
    $('scCollect').textContent=(cp.collect||0).toFixed(1); $('barCollect').style.width=Math.min(100,(cp.collect||0)/15*100)+'%';
    $('scScore').textContent=(cp.score||0).toFixed(1); $('barScore').style.width=Math.min(100,(cp.score||0)/15*100)+'%';
    const w=$('copilotWin'); w.textContent=cp.mode||'—'; w.className='pill '+(cp.mode==='Score'?'ok':(cp.mode==='Collect'?'warn':'no'));
  }
  $('climbBox').style.display=clb.on?'block':'none';
  if(clb.on){ const p=Math.round((clb.pct||0)*100); $('climbPct').textContent=p+'%'; $('barClimb').style.width=Math.min(100,p)+'%'; }
  $('ghostHint').style.display=gh.on?'block':'none';
}

function renderMath(s){
  const r=s.robot, vg=s.virtualGoal, g=s.goalPt, a=s.aim, t=s.turret, sh=s.shooter, hd=s.hood;
  const v=(n,d)=>'<span class="num">'+(+n).toFixed(d===undefined?1:d)+'</span>';
  const vmag=Math.hypot(r.vx,r.vy);
  const lines=[
    '<span class="cm">// inputs (fused pose + field velocity)</span>',
    'pose = ('+v(r.x,1)+', '+v(r.y,1)+', '+v(r.heading*57.2958,0)+'\\u00B0)',
    'v_field = ('+v(r.vx,2)+', '+v(r.vy,2)+')   |v| = '+v(vmag,2)+' m/s',
    '&nbsp;',
    '<span class="cm">// virtual goal — fixed point of  vg = goal \\u2212 v\\u00B7tof</span>',
    'vg   = ('+v(vg.x,2)+', '+v(vg.y,2)+')',
    'lead = goal \\u2212 vg = ('+v(g.x-vg.x,2)+', '+v(g.y-vg.y,2)+')',
    'd    = |vg \\u2212 pose| = '+v(a.distance,2)+' m    tof = '+v(a.shotTime,3)+' s',
    '&nbsp;',
    '<span class="cm">// aim (field frame)</span>',
    '\\u03B8  = atan2(\\u0394y, \\u0394x) = '+v(t.aimField,1)+'\\u00B0  <span class="cm">turret target</span>',
    '\\u03B8\\u0307 = (ry\\u00B7vx \\u2212 rx\\u00B7vy)/|r|\\u00B2 = '+v(a.rateDps,1)+'\\u00B0/s  <span class="cm">velocity FF</span>',
    '&nbsp;',
    '<span class="cm">// lookups (distance \\u2192 setpoint)</span>',
    'rpm  = table(d) = '+v(a.solRpm,0),
    'hood = table(d) = '+v(a.solHood,1)+'\\u00B0',
    'feasible = d \\u2264 '+v(a.maxRange,1)+' m  '+(a.feasible?'<span style="color:var(--ok)">\\u2713</span>':'<span style="color:var(--err)">\\u2717 out of scoring zone</span>'),
  ];
  $('math').innerHTML=lines.map(t=>'<span class="ln">'+t+'</span>').join('');
}

function renderCode(s){
  const g=s.goal.active, fire=s.firing, as=g==='AimShoot', it=g==='Intake',
        auto=s.auto&&s.auto.running;
  const L=(live,html)=>'<span class="ln'+(live?' live':'')+'"><span class="dot"></span>'+html+'</span>';
  const cm=t=>'<span class="cm">'+t+'</span>';
  $('code').innerHTML=[
    L(false,cm('// ── every 20 ms loop ───────────────────────')),
    L(true,'sol = <span class="fn">solver.solve</span>(pose, fieldSpeeds);'),
    L(true,'<span class="fn">turret.track</span>(sol, heading, yawRate);'),
    L(false,'&nbsp;'),
    L(false,cm('// ── goal layer · director.pursue('+g+') ─────')),
    L(as,'<span class="kw">when</span> AIM_SHOOT:'),
    L(as,'&nbsp;&nbsp;<span class="fn">shooter.track</span>(() -&gt; sol.shooterRpm()/60)'),
    L(as,'&nbsp;&nbsp;<span class="fn">hood.track</span>(() -&gt; sol.hoodDegrees())'),
    L(as,'&nbsp;&nbsp;ready = shooter.atSpeed() &amp;&amp; turret.atSetpoint()'),
    L(fire,'&nbsp;&nbsp;<span class="kw">if</span> (ready &amp;&amp; feasible) fireShot();'),
    L(it,'<span class="kw">when</span> INTAKE:'),
    L(it,'&nbsp;&nbsp;<span class="fn">intake.intakeContinuous</span>();'),
    L(false,'&nbsp;'),
    L(false,cm('// ── autonomous plan (20 s) ─────────────────')),
    L(auto,'<span class="fn">runAuto</span>(): driveTo(wp) → director.pursue(action)'),
  ].join('');
}
function esc(t){return (t+'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));}

// --- field drawing ---
// The heightmap header is the authority on how big the field is. These two are the no-map fallback
// and nothing else - reading them while a map is loaded is what put the map out of register.
const FW_FALLBACK=16.54, FH_FALLBACK=8.07;
let FW=FW_FALLBACK, FH=FH_FALLBACK;
// The carpet is painted here rather than baked into the collision layer; see draw().
const CARPET='#1f2740';
// One metres-to-pixels scale for both axes. The old mapping stretched x and y independently to fill
// the canvas, so anything square came out the wrong shape whenever the canvas was not the field's
// aspect. Whatever is left over is centred, and it is sub-pixel once sizeView() has run.
function mpp(c){return Math.min(c.width/FW,c.height/FH);}
function fox(c){return (c.width-FW*mpp(c))/2;}
function foy(c){return (c.height-FH*mpp(c))/2;}
function fx(c,x){return fox(c)+x*mpp(c);}
function fy(c,y){return c.height-foy(c)-y*mpp(c);}
function sizeView(){const c=$('view'); if(!c) return; const h=Math.round(c.width*FH/FW); if(c.height!==h) c.height=h;}

function spawnShot(r){
  if(balls.length>32) return;
  const c=$('view');
  const x0=fx(c,r.x0),y0=fy(c,r.y0),x1=fx(c,r.lx),y1=fy(c,r.ly);
  const dist=Math.hypot(x1-x0,y1-y0);
  // flight time = the real shot's time-of-flight; arc height scales with distance
  balls.push({x0,y0,x1,y1,t:0,dur:Math.max(0.3,Math.min(0.85,r.tof||0.5)),hit:r.hit,done:false,arc:Math.min(44,dist*0.30+8)});
}

let prev=performance.now();
function frame(now){
  const dt=(now-prev)/1000; prev=now;
  if(last) draw(last,dt);
  requestAnimationFrame(frame);
}

// --- collision map, drawn from the CAD-derived heightmap -------------------
  let COLLISION=null, COLLISION_IMG=null;
  fetch('/collision').then(r=>r.json()).then(m=>{
    if(!m||!m.cols) return;
    COLLISION=m;
    // Contract: the file's header is the field, not the constants above. cols*cellMeters and
    // rows*cellMeters are guaranteed to equal these, so the grid lands on the field rect exactly.
    FW=m.lengthMeters; FH=m.widthMeters;
    sizeView();
  }).catch(()=>{});

  /* Rendered once into an offscreen canvas and then blitted, because rasterising ninety thousand
     cells every frame would cost more than everything else in this cockpit put together. */
  function collisionLayer(){
    if(COLLISION_IMG||!COLLISION) return COLLISION_IMG;
    const m=COLLISION, off=document.createElement('canvas');
    off.width=m.cols; off.height=m.rows;
    const g=off.getContext('2d'), img=g.createImageData(m.cols,m.rows);
    for(let i=0;i<m.cols*m.rows;i++){
      const h=m.heightsMillimetres[i], cl=m.clearanceMillimetres?m.clearanceMillimetres[i]:9999;
      let r,gg,b,a;
      if(h>300){ r=70;gg=76;b=92;a=235; }            // structure
      else if(cl<1400){ r=120;gg=90;b=40;a=180; }    // drive under
      else if(h>60){ r=60;gg=90;b=70;a=160; }        // ramp / low feature
      /* Carpet stays transparent on purpose. This layer goes down after the alliance tints, the
         border and the centre line, so an opaque carpet here would rub all three out. draw() paints
         the surface before them instead - it used to paint nothing at all, and 87% of the field was
         the page showing through, which reads as a map that does not fill the canvas. */
      else { r=0;gg=0;b=0;a=0; }
      const at=i*4; img.data[at]=r; img.data[at+1]=gg; img.data[at+2]=b; img.data[at+3]=a;
    }
    g.putImageData(img,0,0);
    COLLISION_IMG=off; return off;
  }

  /* Connected runs of structure in the map, found once. The cockpit's hub and tower constants sit
     over a metre from where the CAD puts the same structures, so a cue that claims to mark one has
     to go looking for it in the map rather than trust the constant. */
  let STRUCTS=null;
  function structures(){
    if(STRUCTS||!COLLISION) return STRUCTS;
    const m=COLLISION, cs=m.cellMeters, h=m.heightsMillimetres,
          seen=new Uint8Array(m.cols*m.rows), out=[];
    for(let i=0;i<seen.length;i++){
      if(seen[i]||h[i]<=300) continue;
      seen[i]=1;
      const stack=[i]; let n=0, c0=m.cols, c1=-1, r0=m.rows, r1=-1;
      while(stack.length){
        const j=stack.pop(), rr=(j/m.cols)|0, cc=j-rr*m.cols;
        n++; if(cc<c0)c0=cc; if(cc>c1)c1=cc; if(rr<r0)r0=rr; if(rr>r1)r1=rr;
        if(cc>0        &&!seen[j-1]     &&h[j-1]     >300){seen[j-1]     =1;stack.push(j-1);}
        if(cc<m.cols-1 &&!seen[j+1]     &&h[j+1]     >300){seen[j+1]     =1;stack.push(j+1);}
        if(rr>0        &&!seen[j-m.cols]&&h[j-m.cols]>300){seen[j-m.cols]=1;stack.push(j-m.cols);}
        if(rr<m.rows-1 &&!seen[j+m.cols]&&h[j+m.cols]>300){seen[j+m.cols]=1;stack.push(j+m.cols);}
      }
      if(n>=20) out.push({x0:c0*cs,y0:r0*cs,x1:(c1+1)*cs,y1:(r1+1)*cs});   // specks are noise
    }
    STRUCTS=out; return out;
  }
  /** The structure the map has nearest a declared point, or null if there is none worth claiming. */
  function mapStruct(px,py){
    const all=structures(); if(!all) return null;
    let best=null, bd=1e9;
    for(const s of all){
      const dx=Math.max(s.x0-px,0,px-s.x1), dy=Math.max(s.y0-py,0,py-s.y1), d=Math.hypot(dx,dy);
      if(d<bd){ bd=d; best=s; }
    }
    // Past this the nearest blob is some unrelated piece of field, and pinning a label to it would
    // just be a different wrong drawing.
    return bd<=2.0?best:null;
  }

  function drawCollision(x,c){
    const layer=collisionLayer(); if(!layer) return;
    /* Straight onto the field rect fx/fy define, so the map cannot drift from the robot drawn over
       it. Scaling the blit by the map's size over the hardcoded constants - as this did - left two
       bare pixels down the right edge and pushed a pixel and a half of map off the top. */
    const w=FW*mpp(c), h=FH*mpp(c);
    x.save(); x.imageSmoothingEnabled=false;
    x.translate(fox(c),foy(c)+h); x.scale(1,-1);   // grid row 0 is field y=0, which is the bottom
    x.drawImage(layer,0,0,w,h);
    x.restore();
  }

  function draw(s,dt){
  const c=$('view'), x=c.getContext('2d'), W=c.width, H=c.height;
  x.clearRect(0,0,W,H);
  // The surface goes down first and everything else - tints, border, centre line, the collision
  // blit - layers over it. Carpet is transparent in the blit precisely so this survives.
  x.fillStyle=CARPET; x.fillRect(0,0,W,H);
  const els=s.fieldEls;
  // alliance zones (REBUILT: red left, blue right)
  if(els){
    x.fillStyle='rgba(233,69,96,0.08)'; x.fillRect(0,0,fx(c,els.allianceZone),H);
    x.fillStyle='rgba(96,165,250,0.08)'; x.fillRect(fx(c,FW-els.allianceZone),0,W,H); }
  // border + dashed center line
  x.strokeStyle='rgba(255,255,255,.18)'; x.lineWidth=2; x.strokeRect(2,2,W-4,H-4);
  x.strokeStyle='rgba(255,255,255,.12)'; x.setLineDash([6,6]); x.beginPath(); x.moveTo(W/2,0); x.lineTo(W/2,H); x.stroke(); x.setLineDash([]);
  // trenches (perimeter lanes robots drive under)
  if(!COLLISION){ x.fillStyle='rgba(255,255,255,0.05)'; x.fillRect(W*0.30,4,W*0.40,13); x.fillRect(W*0.30,H-17,W*0.40,13); }
  // central Fuel field + loading stations / depots / scoring table

  // One field, not two. When the CAD map is loaded it *replaces* the hand-drawn static
  // geometry rather than layering over it - drawing both is what doubled the map.
  if(COLLISION){ drawCollision(x,c); } else { drawFuel(x,W,H); drawStations(x,W,H); }

  // hubs (with hex opening) + towers
  if(els){
    drawHub(x,c,els.redHub,'#e94560','RED HUB',false);
    drawHub(x,c,els.blueHub,'#60a5fa','BLUE HUB', s.match&&s.match.hubActive);
    drawTower(x,c,els.redTower); drawTower(x,c,els.blueTower);
  }
  // autonomous plan path + driven trail
  drawAuto(x,c,s);
  // ghost replay (translucent purple robot)
  if(s.ghost&&s.ghost.replaying){ const gx2=fx(c,s.ghost.x),gy2=fy(c,s.ghost.y),sz=0.9*mpp(c);
    x.globalAlpha=0.42; x.save(); x.translate(gx2,gy2); x.rotate(-s.ghost.hdg);
    x.fillStyle='#a78bfa'; x.strokeStyle='#c4b5fd'; x.lineWidth=2; roundRect(x,-sz/2,-sz/2,sz,sz,6); x.fill(); x.stroke();
    x.restore(); x.globalAlpha=1;
    x.fillStyle='#c4b5fd'; x.font='9px ui-monospace,monospace'; x.fillText('ghost',gx2-13,gy2-sz/2-3); }
  // opponent / defense bot (red)
  if(s.opp&&s.opp.on){ const ox=fx(c,s.opp.x),oy=fy(c,s.opp.y),sz=0.9*mpp(c);
    x.fillStyle='#5e1722'; x.strokeStyle='#e94560'; x.lineWidth=2; roundRect(x,ox-sz/2,oy-sz/2,sz,sz,6); x.fill(); x.stroke();
    x.fillStyle='#ff9bb0'; x.font='bold 11px ui-monospace,monospace'; x.textAlign='center'; x.fillText('D',ox,oy+4); x.textAlign='left'; }
  const gx=fx(c,s.goalPt.x), gy=fy(c,s.goalPt.y);
  // virtual goal (SOTF lead)
  const vgx=fx(c,s.virtualGoal.x), vgy=fy(c,s.virtualGoal.y);
  const moving=Math.abs(s.robot.vx)+Math.abs(s.robot.vy)>0.05;
  if(moving){
    x.strokeStyle='rgba(251,191,36,.7)'; x.setLineDash([4,4]); x.lineWidth=1.5;
    x.beginPath(); x.arc(vgx,vgy,7,0,7); x.stroke();
    x.beginPath(); x.moveTo(gx,gy); x.lineTo(vgx,vgy); x.stroke(); x.setLineDash([]);
    x.fillStyle='#fbbf24'; x.fillText('lead',vgx+9,vgy+3);
  }
  // balls in flight (green = on target, red = will miss); leave a splash on landing
  for(const b of balls){ b.t+=dt; if(b.t>=b.dur && !b.done){ b.done=true; splashes.push({x:b.x1,y:b.y1,t:0,hit:b.hit}); } }
  balls=balls.filter(b=>b.t<b.dur);
  for(const b of balls){ const u=b.t/b.dur; const bx=b.x0+(b.x1-b.x0)*u, by=b.y0+(b.y1-b.y0)*u - Math.sin(u*Math.PI)*(b.arc||22);
    x.fillStyle=b.hit?'#ffd166':'#f87171'; x.shadowColor=b.hit?'#ffd166':'#f87171'; x.shadowBlur=6;
    x.beginPath(); x.arc(bx,by,4.5,0,7); x.fill(); x.shadowBlur=0; }
  // splashes (hit ring on the goal / miss mark where it landed)
  for(const sp of splashes){ sp.t+=dt; }
  splashes=splashes.filter(sp=>sp.t<0.6);
  for(const sp of splashes){ const u=sp.t/0.6, rad=6+u*12;
    x.globalAlpha=1-u; x.strokeStyle=sp.hit?'#4ade80':'#f87171'; x.lineWidth=2;
    x.beginPath(); x.arc(sp.x,sp.y,rad,0,7); x.stroke();
    if(!sp.hit){ x.beginPath(); x.moveTo(sp.x-5,sp.y-5); x.lineTo(sp.x+5,sp.y+5); x.moveTo(sp.x+5,sp.y-5); x.lineTo(sp.x-5,sp.y+5); x.stroke(); }
    x.globalAlpha=1; }
  // robot (sized from its real footprint, rotated by heading)
  const rx=fx(c,s.robot.x), ry=fy(c,s.robot.y), size=(s.robotR||0.45)*2*mpp(c), hd=-s.robot.heading;
  x.save(); x.translate(rx,ry); x.rotate(hd);
  x.fillStyle='#26304f'; x.strokeStyle='rgba(255,255,255,.25)'; x.lineWidth=2;
  roundRect(x,-size/2,-size/2,size,size,7); x.fill(); x.stroke();
  // intake bar (front = +x)
  x.fillStyle=(s.goal.active==='Intake')?'#fbbf24':'#5a6488';
  roundRect(x,size/2-3,-size/2+5,6,size-10,3); x.fill();
  x.restore();
  // turret barrel (absolute field bearing)
  const aim=s.shooter.atSpeed&&s.turret.atSetpoint, spin=s.shooter.rpm>200;
  const tcol=aim?'#4ade80':(spin?'#fbbf24':'#8a8aa8');
  x.fillStyle='#1b2240'; x.strokeStyle=tcol; x.lineWidth=3;
  x.beginPath(); x.arc(rx,ry,12,0,7); x.fill(); x.stroke();
  const a=s.turret.field*Math.PI/180, bl=30;
  x.strokeStyle=tcol; x.lineWidth=6; x.lineCap='round';
  x.beginPath(); x.moveTo(rx,ry); x.lineTo(rx+Math.cos(a)*bl, ry-Math.sin(a)*bl); x.stroke();
  // muzzle flash
  if(s.firing){ x.fillStyle='rgba(249,168,37,.85)'; x.beginPath(); x.arc(rx+Math.cos(a)*bl, ry-Math.sin(a)*bl,8,0,7); x.fill(); }
  // velocity vector (the v that drives the SOTF lead)
  const vmag=Math.hypot(s.robot.vx,s.robot.vy);
  if(vmag>0.15){
    const sc=16, ex=rx+s.robot.vx*sc, ey=ry-s.robot.vy*sc, ah=Math.atan2(ey-ry,ex-rx);
    x.strokeStyle='#60a5fa'; x.lineWidth=2.5; x.beginPath(); x.moveTo(rx,ry); x.lineTo(ex,ey); x.stroke();
    x.fillStyle='#60a5fa'; x.beginPath(); x.moveTo(ex,ey);
    x.lineTo(ex-10*Math.cos(ah-0.4),ey-10*Math.sin(ah-0.4)); x.lineTo(ex-10*Math.cos(ah+0.4),ey-10*Math.sin(ah+0.4)); x.closePath(); x.fill();
    x.fillStyle='#9ec5ff'; x.font='10px ui-monospace,monospace'; x.fillText('v='+vmag.toFixed(1),ex+5,ey-2);
  }
  // hud
  x.fillStyle='#8a8aa8'; x.font='11px ui-monospace,monospace';
  x.fillText('pose '+s.robot.x.toFixed(1)+', '+s.robot.y.toFixed(1)+' m',10,18);
  x.fillStyle=aim?'#4ade80':'#8a8aa8'; x.fillText(aim?'READY TO SHOOT':(spin?'spinning up…':'idle'),10,H-12);
}
function roundRect(x,X,Y,w,h,r){x.beginPath();x.moveTo(X+r,Y);x.arcTo(X+w,Y,X+w,Y+h,r);x.arcTo(X+w,Y+h,X,Y+h,r);
  x.arcTo(X,Y+h,X,Y,r);x.arcTo(X,Y,X+w,Y,r);x.closePath();}
function hex(x,cx,cy,r){x.beginPath();for(let i=0;i<6;i++){const a=Math.PI/6+i*Math.PI/3,px=cx+Math.cos(a)*r,py=cy+Math.sin(a)*r;i?x.lineTo(px,py):x.moveTo(px,py);}x.closePath();}
function drawHub(x,c,h,color,label,active){
  if(!h) return;
  /* The map draws the hub itself, and it draws it well over a metre from this constant - so with a
     map loaded the filled box was an obstacle you could drive through sitting next to one you could
     not see. What the map has no notion of is the live hub state and whose hub it is, so those two
     stay: a dashed outline on the structure the map really has, plus the hex and the label. */
  const st=mapStruct(h.x,h.y);
  let hx,hy,r;
  if(st){
    const L=fx(c,st.x0), R=fx(c,st.x1), T=fy(c,st.y1), B=fy(c,st.y0);
    hx=(L+R)/2; hy=(T+B)/2; r=Math.min(R-L,B-T)/2;
    x.strokeStyle=color; x.lineWidth=1.5; x.setLineDash([4,3]);
    roundRect(x,L,T,R-L,B-T,5); x.stroke(); x.setLineDash([]);
  }else{
    hx=fx(c,h.x); hy=fy(c,h.y); r=h.r*mpp(c);
    x.fillStyle='rgba(255,255,255,0.05)';                       // bumps (flank, traversable)
    x.fillRect(hx-r*0.7,hy-r*2.4,r*1.4,r*0.85); x.fillRect(hx-r*0.7,hy+r*1.55,r*1.4,r*0.85);
    x.fillStyle=color+'33'; x.strokeStyle=color; x.lineWidth=2; roundRect(x,hx-r,hy-r,r*2,r*2,5); x.fill(); x.stroke();
  }
  hex(x,hx,hy,r*0.62); x.strokeStyle=active?'#4ade80':'rgba(255,255,255,0.4)'; x.lineWidth=2; x.stroke();
  if(active){ x.fillStyle='rgba(74,222,128,0.22)'; x.fill(); }
  x.fillStyle=color; x.font='10px ui-monospace,monospace'; x.fillText(label,hx-r,hy-r-6);
}
function drawTower(x,c,t){ if(!t) return;
  // Unlike the hub there is no live state or label to preserve here, so once the map is loaded the
  // map is the whole drawing.
  if(COLLISION) return;
  const tx=fx(c,t.x),ty=fy(c,t.y),r=t.r*mpp(c);
  x.fillStyle='rgba(30,34,64,0.85)'; x.strokeStyle='rgba(200,200,210,0.6)'; x.lineWidth=2; roundRect(x,tx-r,ty-r,r*2,r*2,4); x.fill(); x.stroke();
  x.strokeStyle='rgba(255,255,255,0.4)'; x.lineWidth=1; for(let i=-1;i<=1;i++){x.beginPath();x.moveTo(tx-r*0.6,ty+i*r*0.5);x.lineTo(tx+r*0.6,ty+i*r*0.5);x.stroke();}
}
function drawFuel(x,W,H){ const x0=W*0.455,y0=H*0.17,w=W*0.09,h=H*0.66;
  x.fillStyle='rgba(249,168,37,0.06)'; x.fillRect(x0,y0,w,h);
  x.strokeStyle='rgba(255,255,255,0.07)'; x.lineWidth=1; x.beginPath(); x.moveTo(x0,H/2); x.lineTo(x0+w,H/2); x.stroke();
  x.fillStyle='#f9a825'; const cols=5,rows=17;
  for(let i=0;i<cols;i++)for(let j=0;j<rows;j++){x.beginPath();x.arc(x0+(i+0.5)*w/cols,y0+(j+0.5)*h/rows,1.9,0,7);x.fill();}
}
function drawStations(x,W,H){
  // depots (barriers along each alliance wall)
  x.fillStyle='rgba(233,69,96,0.22)'; x.fillRect(3,H*0.31,7,H*0.38);
  x.fillStyle='rgba(96,165,250,0.22)'; x.fillRect(W-10,H*0.31,7,H*0.38);
  // loading-station Fuel racks at the alliance ends (human player feeds)
  rack(x,W*0.05,H*0.74,'#e94560'); rack(x,W*0.07,H*0.09,'#e94560');
  rack(x,W*0.95,H*0.26,'#60a5fa'); rack(x,W*0.93,H*0.91,'#60a5fa');
  // scoring table along the near wall
  x.fillStyle='rgba(16,20,38,0.92)'; roundRect(x,W*0.36,H-10,W*0.28,8,2); x.fill();
  x.fillStyle='#7782a8'; x.font='8px ui-monospace,monospace'; x.textAlign='center'; x.fillText('SCORING TABLE',W*0.5,H-4); x.textAlign='left';
}
function rack(x,cx,cy,col){
  x.strokeStyle=col; x.globalAlpha=0.7; x.lineWidth=1; roundRect(x,cx-13,cy-9,26,18,3); x.stroke(); x.globalAlpha=1;
  x.fillStyle='#f9a825'; for(let i=0;i<4;i++)for(let j=0;j<3;j++){x.beginPath();x.arc(cx-9.5+i*6.3,cy-6+j*6,1.9,0,7);x.fill();}
}
function drawAuto(x,c,s){ const au=s.auto; if(!au) return;
  if(au.trail&&au.trail.length>1){ x.strokeStyle='rgba(255,255,255,0.28)'; x.lineWidth=2; x.beginPath();
    au.trail.forEach((p,i)=>{const px=fx(c,p.x),py=fy(c,p.y); i?x.lineTo(px,py):x.moveTo(px,py);}); x.stroke(); }
  if(au.plan){
    x.strokeStyle='rgba(167,139,250,0.55)'; x.lineWidth=1.5; x.setLineDash([5,4]); x.beginPath();
    au.plan.forEach((p,i)=>{const px=fx(c,p.x),py=fy(c,p.y); i?x.lineTo(px,py):x.moveTo(px,py);}); x.stroke(); x.setLineDash([]);
    au.plan.forEach((p,i)=>{const px=fx(c,p.x),py=fy(c,p.y);
      x.fillStyle=p.mode===1?'#f9a825':(p.mode===2?'#4ade80':'#a78bfa');
      x.beginPath(); x.arc(px,py,(au.running&&i===au.idx)?6:4,0,7); x.fill();
      if(au.running&&i===au.idx){ x.strokeStyle='#fff'; x.lineWidth=2; x.beginPath(); x.arc(px,py,9,0,7); x.stroke(); }});
  }
}

sizeView(); setInterval(poll,55); requestAnimationFrame(frame); poll();
</script>
</body>
</html>
""";
}
