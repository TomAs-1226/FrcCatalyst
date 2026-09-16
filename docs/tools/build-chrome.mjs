// Put the identity's tokens into the tool chrome.
//
//   node docs/tools/build-chrome.mjs           # rewrite the block in tool.css
//   node docs/tools/build-chrome.mjs --check   # fail if it has drifted (what the test runs)
//
// The tools link `../tool.css` and nothing else, because the site keeps the identity at
// `docs/assets/` and the desktop app copies the tools somewhere else entirely — one relative path
// cannot reach the same stylesheet from both. So the tokens are copied, and copied by a script with
// a check on it rather than by hand.

import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const identity = resolve(here, "..", "assets", "identity.css");
const chrome = join(here, "tool.css");

const OPEN = "/* >>> identity tokens — generated, do not edit by hand */";
const CLOSE = "/* <<< identity tokens */";

/** The `:root { … }` block, taken whole so a token added to the identity arrives here too. */
export function rootBlock(css) {
  const start = css.indexOf(":root {");
  if (start < 0) throw new Error("identity.css has no :root block");
  const end = css.indexOf("\n}", start);
  if (end < 0) throw new Error("identity.css's :root block is not closed");
  return css.slice(start, end + 2);
}

export function rebuild(chromeCss, rootCss) {
  const open = chromeCss.indexOf(OPEN);
  const close = chromeCss.indexOf(CLOSE);
  if (open < 0 || close < 0) throw new Error("tool.css has lost its generated markers");
  return chromeCss.slice(0, open + OPEN.length) + "\n" + rootCss + "\n" + chromeCss.slice(close);
}

const wanted = rebuild(readFileSync(chrome, "utf8"), rootBlock(readFileSync(identity, "utf8")));
const current = readFileSync(chrome, "utf8");

if (process.argv.includes("--check")) {
  if (wanted !== current) {
    console.error("tool.css's tokens have drifted from docs/assets/identity.css");
    console.error("Run `node docs/tools/build-chrome.mjs` to bring them back in step.");
    process.exit(1);
  }
  console.log("tool.css is in step with the identity");
} else if (wanted === current) {
  console.log("tool.css is already in step with the identity");
} else {
  writeFileSync(chrome, wanted);
  console.log("tool.css: tokens rebuilt from docs/assets/identity.css");
}
