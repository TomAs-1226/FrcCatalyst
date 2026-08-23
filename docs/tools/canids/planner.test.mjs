// The CAN ID planner's logic, checked outside a browser.
//
// Run with:  node --test docs/tools/
//
// The tool is a single self-contained HTML file, so this pulls its script out and runs it against a
// DOM stub thin enough to be obviously not a browser. Only the pure parts are checked - the bus
// model, the contention advice, and the two generators. Layout is not tested here and is not
// claimed to be.
//
// What makes this worth having: the tool generates CANIds.java, which is compiled against the
// library. If the generator and CANRegistry drift apart, the failure lands on a team at build time
// with a message about a method signature, and nothing points back at the planner.

import fs from "node:fs";
import assert from "node:assert/strict";
import vm from "node:vm";
import test from "node:test";
import { fileURLToPath } from "node:url";
import path from "node:path";

const here = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.join(here, "index.html"), "utf8");
const script = html.match(/<script[^>]*>([\s\S]*?)<\/script>/)[1];

// --- the thinnest DOM that lets the file finish loading ---------------------
const el = () => ({
  value: "", textContent: "", innerHTML: "", className: "", style: {},
  classList: { add() {}, remove() {}, toggle() {} },
  appendChild() {}, addEventListener() {}, querySelectorAll: () => [],
  set onclick(_) {}, set onchange(_) {}, dataset: {},
});
const store = new Map();
const sandbox = {
  console,
  document: {
    getElementById: () => el(),
    querySelectorAll: () => [],
    createElement: el,
    addEventListener() {},
    body: el(),
  },
  window: { addEventListener() {} },
  localStorage: {
    getItem: k => store.get(k) ?? null,
    setItem: (k, v) => store.set(k, v),
    removeItem: k => store.delete(k),
  },
  alert() {}, confirm: () => true, FileReader: class {}, Blob: class {},
  URL: { createObjectURL: () => "", revokeObjectURL() {} },
  navigator: { clipboard: { writeText: async () => {} } },
};
sandbox.globalThis = sandbox;
vm.createContext(sandbox);
vm.runInContext(script, sandbox);

// The tool keeps its state in a module-level `devices`; reach it through the context.
const ctx = sandbox;
const run = expr => vm.runInContext(expr, ctx);
// vm values carry the sandbox realm's prototypes, so deepEqual sees two identical arrays as
// different types. Round-tripping through JSON brings them back into this realm.
const runJson = expr => JSON.parse(JSON.stringify(run(expr)));

// --- the bus model ----------------------------------------------------------

test("the five Systemcore buses replace the roboRIO bus", () => {
assert.deepEqual(
  runJson("ALL_BUSES"),
  ["can_s0", "can_s1", "can_s2", "can_s3", "can_s4", "canivore"],
  "five Systemcore buses plus CANivore",
);

assert.equal(run('normaliseBus("rio")'), "can_s0", "the roboRIO bus maps onto Catalyst's default");
assert.equal(run('normaliseBus("")'), "can_s0", "so does the empty-string bus");
assert.equal(run('normaliseBus("can_s3")'), "can_s3", "a real bus is left alone");
assert.equal(run('normaliseBus("Drivebase")'), "canivore", "an unknown name is a CANivore");
});

// --- contention advice ------------------------------------------------------
const plan = devices => runJson(`devices = ${JSON.stringify(devices)}; contentionWarnings()`);
const dev = (i, bus) => ({ name: `M${i}`, id: i, type: "Kraken X60", bus });

test("advice is given only when it would change the wiring", () => {
  assert.deepEqual(plan([]), [], "an empty plan has nothing to say");

assert.deepEqual(
  plan(Array.from({ length: 8 }, (_, i) => dev(i, "can_s0"))),
  [],
  "eight devices on one bus is ordinary and must not nag",
);

const piled = plan(Array.from({ length: 16 }, (_, i) => dev(i, "can_s0")));
assert.equal(piled.length, 1, "sixteen on one bus is worth mentioning");
assert.match(piled[0], /five buses/);

const split = plan([
  ...Array.from({ length: 8 }, (_, i) => dev(i, "can_s0")),
  ...Array.from({ length: 8 }, (_, i) => dev(i + 20, "can_s1")),
]);
assert.equal(split.length, 1, "a split across a paired bus is the interesting case");
assert.match(split[0], /share an SPI controller/);
assert.match(split[0], /can_s2/, "and it should name a bus on a different controller");

assert.deepEqual(
  plan([
    ...Array.from({ length: 8 }, (_, i) => dev(i, "can_s0")),
    ...Array.from({ length: 8 }, (_, i) => dev(i + 20, "can_s3")),
  ]),
  [],
  "the same split across unpaired buses is exactly right and must say nothing",
);
});

// --- generated Java ---------------------------------------------------------
const java = devs => {
  run(`devices = ${JSON.stringify(devs)}`);
  return run("buildJava(devices.map((d,i)=>({...d,_i:i})).sort((a,b)=>a.bus.localeCompare(b.bus)||a.id-b.id), detectConflicts())");
};

test("the generated Java matches what CANRegistry accepts", () => {
  const generated = java([
  { name: "FrontLeftDrive", id: 1, type: "Kraken X60", bus: "can_s0" },
  { name: "ArmMaster", id: 20, type: "Kraken X60", bus: "can_s2" },
]);

assert.match(generated, /public static final String CAN_S0\s+= "can_s0";/, "a constant per bus used");
assert.match(generated, /public static final String CAN_S2\s+= "can_s2";/);
assert.doesNotMatch(generated, /CAN_S1|CAN_S3|CAN_S4|CANIVORE/, "and none for buses not used");
assert.doesNotMatch(generated, /RIO/, "nothing should mention a roboRIO");

// The registration calls are what CANRegistry actually receives.
assert.match(generated, /CANRegistry\.register\("FrontLeftDrive", FRONT_LEFT_DRIVE, CAN_S0, "Kraken X60"\);/);
assert.match(generated, /CANRegistry\.register\("ArmMaster", ARM_MASTER, CAN_S2, "Kraken X60"\);/);
assert.match(generated, /shares a controller with can_s1/, "the pairing is recorded where it is read");
});

// --- generated text ---------------------------------------------------------
const text = devs => {
  run(`devices = ${JSON.stringify(devs)}`);
  return run("buildText(devices.map((d,i)=>({...d,_i:i})).sort((a,b)=>a.bus.localeCompare(b.bus)||a.id-b.id), detectConflicts())");
};

test("the plain listing groups by bus, in order", () => {
  const listing = text([
  { name: "A", id: 1, type: "Kraken X60", bus: "can_s4" },
  { name: "B", id: 2, type: "Kraken X60", bus: "can_s0" },
]);
assert.match(listing, /\[can_s0\]/);
assert.match(listing, /\[can_s4\]/);
assert.ok(listing.indexOf("[can_s0]") < listing.indexOf("[can_s4]"), "buses listed in order");
});

// --- conflicts are still per-bus -------------------------------------------
test("a duplicate id is a conflict only on the same bus", () => {
  const sameIdDifferentBuses = runJson(`devices = ${JSON.stringify([
  { name: "A", id: 5, type: "Kraken X60", bus: "can_s0" },
  { name: "B", id: 5, type: "Kraken X60", bus: "can_s1" },
])}; [...detectConflicts()]`);
assert.deepEqual(sameIdDifferentBuses, [], "the same id on two buses is legal and is the point of having five");

const sameIdSameBus = run(`devices = ${JSON.stringify([
  { name: "A", id: 5, type: "Kraken X60", bus: "can_s2" },
  { name: "B", id: 5, type: "Kraken X60", bus: "can_s2" },
])}; [...detectConflicts()].length`);
assert.equal(sameIdSameBus, 2, "and on one bus it is still a conflict");
});
