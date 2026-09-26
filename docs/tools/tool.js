/**
 * The tool chrome's behaviour: the mark, the way back, and the press.
 *
 *     <script src="../tool.js" defer></script>
 *
 * Every tool is one self-contained file, and this is the half of it that is the same in all of them.
 * Nothing here knows what any tool does; it does not touch a tool's own state, and a tool that loads
 * it and ignores it works exactly as before.
 *
 * The way back is the part worth explaining. A tool is opened in two places — the documentation site,
 * where it is a page under `/tools/`, and the desktop app, where it is a frame inside the shell. The
 * link used to be `../` in both, which on the site is the tools index and in the app is a folder with
 * no index: a blank page, and no way out of it but the sidebar. So: in a frame it asks the shell to
 * go to its own Tools view, and outside one it is an ordinary link.
 */

(() => {
  "use strict";

  const BOLT = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M13.5 2 5 13h5l-1.5 9L19 10h-5.5z" fill="currentColor"/></svg>';
  const IN_FRAME = window.parent !== window;
  const reduced = () => window.matchMedia?.("(prefers-reduced-motion: reduce)").matches;

  /** The bolt, in front of the tool's name, unless the page already drew one. */
  function addMark(header) {
    if (header.querySelector(".mark")) return;
    const mark = document.createElement("span");
    mark.className = "mark";
    mark.innerHTML = BOLT;
    header.insertBefore(mark, header.firstChild);
  }

  /**
   * The way back.
   *
   * The message is deliberately small and named: the shell listens for `catalyst` and ignores
   * anything else, and a tool page cannot be made to navigate the app anywhere but its own tools
   * list by sending it.
   */
  function wireBack(header) {
    const back = header.querySelector(".back");
    if (!back) return;
    if (!back.textContent.trim()) back.textContent = "Catalyst tools";
    back.insertAdjacentHTML("afterbegin", '<span aria-hidden="true">←</span> ');

    if (!IN_FRAME) return;
    // Inside the app the href would land on a folder with no index, so it stops being a link.
    back.setAttribute("role", "button");
    back.removeAttribute("href");
    back.addEventListener("click", (e) => {
      e.preventDefault();
      window.parent.postMessage({ catalyst: "tools" }, "*");
    });
  }

  /**
   * Bezel's press answer: a state layer that grows from the point pressed, on pointer-down rather
   * than on click, because the control should acknowledge the finger and not the release.
   */
  function pressLayer(el, event) {
    if (reduced()) return;
    const r = el.getBoundingClientRect();
    const x = (event.clientX ?? r.left + r.width / 2) - r.left;
    const y = (event.clientY ?? r.top + r.height / 2) - r.top;
    const reach = Math.max(
      Math.hypot(x, y),
      Math.hypot(r.width - x, y),
      Math.hypot(x, r.height - y),
      Math.hypot(r.width - x, r.height - y),
    );

    const layer = document.createElement("span");
    layer.className = "state-layer";
    layer.style.left = `${x}px`;
    layer.style.top = `${y}px`;
    document.body.appendChild(layer);       // measured against the button, drawn inside it
    el.appendChild(layer);

    const style = getComputedStyle(document.documentElement);
    const ms = (parseFloat(style.getPropertyValue("--cat-dur-layer")) || 0.155) * 1000;
    const ease = style.getPropertyValue("--cat-ease-layer").trim() || "linear";

    layer
      .animate(
        [
          { width: "0px", height: "0px", opacity: 0.16 },
          { width: `${reach * 2}px`, height: `${reach * 2}px`, opacity: 0 },
        ],
        { duration: ms, easing: ease, fill: "forwards" },
      )
      .finished.catch(() => {})
      .finally(() => layer.remove());
  }

  function wirePresses() {
    document.addEventListener("pointerdown", (e) => {
      const el = e.target.closest("button, .btn, .back, .tag");
      if (!el || el.disabled) return;
      const position = getComputedStyle(el).position;
      if (position === "static") el.style.position = "relative";
      pressLayer(el, e);
    });
  }

  function start() {
    const header = document.querySelector("header");
    if (header) {
      addMark(header);
      wireBack(header);
    }
    wirePresses();
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", start);
  else start();
})();
