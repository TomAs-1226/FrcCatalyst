// Every version string on the docs site agrees with the library it documents.
//
// Run with:  node --test docs/tools/
//
// These badges are hand-written in a dozen files and had drifted a full major version - the tools
// said v1.4.0 while the library was 2.0.0-alpha.1, and the home page hero said "v1.4.0 . stable"
// directly above a banner explaining that this is a beta that has never run on hardware.
//
// Nothing about that breaks a build, which is exactly why it survived. A number that is wrong in
// only one place is a typo; the same wrong number in eleven places reads as authoritative.
//
// Rather than wire Jekyll templating into the tools - they are deliberately standalone files that
// work when saved to disk - the drift is caught here.

import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import assert from "node:assert/strict";
import { fileURLToPath } from "node:url";

const toolsDir = path.dirname(fileURLToPath(import.meta.url));
const docsDir = path.dirname(toolsDir);
const repoRoot = path.dirname(docsDir);

/** The one source of truth: whatever build.gradle publishes. */
function libraryVersion() {
  const gradle = fs.readFileSync(path.join(repoRoot, "build.gradle"), "utf8");
  const m = gradle.match(/^version\s*=\s*['"]([^'"]+)['"]/m);
  assert.ok(m, "could not read the version out of build.gradle");
  return m[1];
}

function toolPages() {
  return fs.readdirSync(toolsDir)
    .map(name => path.join(toolsDir, name, "index.html"))
    .filter(p => fs.existsSync(p));
}

test("every tool's badge names the version of the library it generates code for", () => {
  const version = libraryVersion();
  const wrong = [];

  for (const page of toolPages()) {
    // Read as bytes: one of these pages trips git's binary heuristic, and decoding it as UTF-8
    // would fail on a check that has nothing to do with encoding.
    const html = fs.readFileSync(page).toString("latin1");
    const m = html.match(/class="badge"[^>]*>\s*v([^<\s]+)/);
    if (!m) continue;               // a tool without a badge is fine; a wrong one is not
    if (m[1] !== version) {
      wrong.push(`${path.relative(repoRoot, page)}: v${m[1]}`);
    }
  }

  assert.deepEqual(wrong, [], `expected v${version}`);
});

test("the home page hero names the same version", () => {
  const version = libraryVersion();
  const hero = fs.readFileSync(path.join(docsDir, "_includes", "hero3d.html"), "utf8");
  const m = hero.match(/class="tag">\s*v([^\s<]+)/);

  assert.ok(m, "the hero should carry a version tag");
  assert.equal(m[1], version);
});

test("the hero does not call a beta build stable", () => {
  // The specific mistake that was live: a "stable" tag sitting above the banner that explains this
  // documents an alpha WPILib on a beta OS.
  const version = libraryVersion();
  const hero = fs.readFileSync(path.join(docsDir, "_includes", "hero3d.html"), "utf8");
  const tag = hero.match(/class="tag">([^<]*)</)[1];

  if (/alpha|beta|rc/i.test(version)) {
    assert.doesNotMatch(tag, /stable/i, `"${tag.trim()}" claims stable for a pre-release version`);
  }
});

test("the README banner names the current version", () => {
  // It had drifted to v1.9.2 while the library was on 1.12.0 - the first thing anyone sees on the
  // repository page, and two releases out of date. Nothing reads it, so nothing caught it.
  const version = libraryVersion();
  const svg = fs.readFileSync(path.join(docsDir, "assets", "banner.svg"), "utf8");
  // Match a version-shaped string with or without the leading v: the banners write it both ways,
  // and requiring one spelling would fail a banner that is perfectly correct.
  const m = svg.match(/>v?(\d+\.\d+\.\d+[^<]*)</);

  assert.ok(m, "the banner should carry a version badge");
  assert.equal(m[1], version);
});

test("the banner does not advertise the wrong season", () => {
  // It said WPILib 2026 on a branch that targets 2027. A banner is the one piece of documentation
  // read by people who read no documentation.
  const svg = fs.readFileSync(path.join(docsDir, "assets", "banner.svg"), "utf8");
  const seasons = [...svg.matchAll(/WPILib (\d{4})/g)].map(x => x[1]);

  assert.ok(seasons.length, "the banner should say which WPILib it is for");
  for (const season of seasons) {
    assert.equal(season, "2027", `banner says WPILib ${season}`);
  }
});

test("the AdvantageScope bundles name the version whose topics they match", () => {
  // Not cosmetic. 2.x renamed the swerve topics, so a bundle still claiming 1.7.0 tells the reader
  // its layout predates that rename - which would mean the layout resolves nothing.
  const version = libraryVersion();
  const dir = path.join(toolsDir, "advantagescope");
  const wrong = [];

  for (const name of fs.readdirSync(dir).filter(n => n.endsWith(".json"))) {
    const json = JSON.parse(fs.readFileSync(path.join(dir, name), "utf8"));
    if (json.catalystVersion && json.catalystVersion !== version) {
      wrong.push(`${name}: ${json.catalystVersion}`);
    }
  }

  assert.deepEqual(wrong, [], `expected ${version}`);
});

/** WPILib releases that are on no public maven, so JitPack cannot build this library against them. */
const SOURCE_ONLY_WPILIB = ["2027.0.0-alpha-6"];

function wpilibVersion() {
  const gradle = fs.readFileSync(path.join(repoRoot, "build.gradle"), "utf8");
  return gradle.match(/wpilibVersion\s*=\s*['"]([^'"]+)['"]/)?.[1] ?? "";
}

test("the published vendordep installs a version that exists", () => {
  // The file teams actually install, so it must name something installable.
  //
  // On most lines that means the library version. On a source-build-only line it means the opposite:
  // JitPack cannot build the library at all there, so the vendordep deliberately stays at the newest
  // tag it could build, and matching the library version would install a jar that does not exist -
  // the very failure this test exists to prevent, inverted. When it lags, the install page has to say
  // which version it really installs.
  const version = libraryVersion();
  const vendordep = JSON.parse(
    fs.readFileSync(path.join(docsDir, "vendordep", "FrcCatalyst.json"), "utf8"));

  if (SOURCE_ONLY_WPILIB.includes(wpilibVersion()) && vendordep.version !== version) {
    const install = fs.readFileSync(
      path.join(docsDir, "getting-started", "installation.md"), "utf8");
    assert.ok(install.includes(vendordep.version),
      `the vendordep installs ${vendordep.version} while the library is ${version}; ` +
      `installation.md must name ${vendordep.version} and explain why`);
  } else {
    assert.equal(vendordep.version, version);
  }

  // Whatever it names, the file must agree with itself.
  for (const dep of [...(vendordep.javaDependencies ?? []), ...(vendordep.jniDependencies ?? [])]) {
    if (dep.groupId && dep.groupId.includes("catalyst")) {
      assert.equal(dep.version.replace(/^v/, ""), vendordep.version, `${dep.artifactId}`);
    }
  }
});

test("the beta vendordep points at its own URL, not the stable one", () => {
  // WPILib uses jsonUrl to check for updates. Pointing it at the stable path makes a 2027 project
  // quietly "update" itself down to the 2026 library.
  const vendordep = JSON.parse(
    fs.readFileSync(path.join(docsDir, "vendordep", "FrcCatalyst.json"), "utf8"));
  const config = fs.readFileSync(path.join(docsDir, "_config.yml"), "utf8");
  const baseurl = config.match(/^baseurl:\s*"([^"]*)"/m)[1];

  assert.ok(vendordep.jsonUrl.includes(baseurl + "/vendordep/"),
    `jsonUrl ${vendordep.jsonUrl} should sit under this site's baseurl ${baseurl}`);
});

test("the vendordep names the year field GradleRIO 2027 actually reads", () => {
  // 2027 renamed this field. A vendordep carrying the 2026 spelling is not merely ignored - the
  // GradleRIO plugin refuses to apply at all, so the project fails before compiling anything:
  //
  //   Vendor Dependency FrcCatalyst has invalid year null. Expected to be 2027_alpha5.
  //
  // Nothing in that message says "your vendordep uses the wrong key", and the version and URL
  // checks above both pass on a file that fails this way. Found by installing this vendordep into
  // a real 2027 project.
  const vendordep = JSON.parse(
    fs.readFileSync(path.join(docsDir, "vendordep", "FrcCatalyst.json"), "utf8"));

  assert.equal(vendordep.frcYear, undefined,
    "frcYear is the 2026 spelling; 2027 reads wpilibYear and rejects the file outright");
  assert.equal(vendordep.wpilibYear, "2027_alpha5",
    "GradleRIO compares this string exactly against the WPILib release it was built for");
});
