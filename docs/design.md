---
layout: default
title: Design
nav_order: 8.5
---

# Design
{: .no_toc }

The Catalyst house visual identity — for anyone building a Catalyst surface: the desktop app, the
Console, a tool page, or a future one.
{: .fs-6 .fw-300 }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

## What this is

The identity is the launch material — `docs/assets/banner.svg` and `docs/assets/release-2.0.svg` —
turned into reusable tokens and primitives. Every colour, size, corner and curve in
[`docs/assets/identity.css`](assets/identity.css) is taken from those two drawings, or from the
Bezel and Detent tools, and nothing is invented at a call site. `docs/assets/motion.js` carries the
same springs into script, for the two things CSS cannot do on its own.

Three rules hold the look together:

1. **Depth comes from the ground, not from shadows.** Surfaces step up in tone and separate with a
   hairline. Only things that float — a dialog, the command palette, a toast — are glass, and glass
   never sits on glass.
2. **One signal colour.** Crimson marks the primary action, progress and fault, and nothing else. A
   screen with two accents has none.
3. **Motion is a spring, by role.** Six curves, named for what they do; a transition that is none of
   them does not belong.

Two departures from Bezel are deliberate, so they read as choices and not drift:

- Type is Segoe UI Variable / Cascadia, not Google Sans Flex / Code. The launch graphics are drawn
  in it, it ships with Windows, and these apps must render with no network and no webfont.
- Corners are squarer than Bezel's shape scale in Catalyst Console, which is an instrument: it
  re-points the radius tokens to 2px and changes nothing else.

## Token reference

Every custom property `identity.css` defines under `:root`, grouped the way the file's own comments
group them.

### Ground

The banner's diagonal, flattened for UI: the gradient is for the window, the flat tone for anything
that has to match a canvas or a video frame.

| Token | Value | Use it for |
|---|---|---|
| `--cat-ground` | `#0d0e12` | The flat ground tone — video frames, canvases, anywhere the gradient can't apply |
| `--cat-ground-top` | `#191a20` | Top stop of the ground gradient |
| `--cat-ground-mid` | `#101116` | Middle stop of the ground gradient |
| `--cat-ground-bottom` | `#08090b` | Bottom stop of the ground gradient |
| `--cat-ground-grad` | `linear-gradient(145deg, var(--cat-ground-top) 0%, var(--cat-ground-mid) 55%, var(--cat-ground-bottom) 100%)` | The window background |
| `--cat-key-light` | `radial-gradient(60% 55% at 76% 40%, rgba(233, 69, 96, .10), rgba(233, 69, 96, .02) 45%, transparent 100%)` | The crimson key light from the launch canvas, upper right. Decoration only — never put text where it lands without checking the pair |
| `--cat-rim-light` | `radial-gradient(52% 52% at 16% 16%, rgba(255, 255, 255, .05), transparent 100%)` | The white rim light, upper left |

### Surfaces

| Token | Value | Use it for |
|---|---|---|
| `--cat-surface-1` | `#131418` | Cards, panels, the editor's ground |
| `--cat-surface-2` | `#1a1c21` | Chips, inputs, rows that lift |
| `--cat-surface-3` | `#22242a` | Hover and pressed |
| `--cat-line` | `#23252b` | The ordinary border |
| `--cat-line-strong` | `#2f323a` | A border that has to be seen: chips, focus |
| `--cat-hair` | `rgba(255, 255, 255, .06)` | A hairline separator |

### Ink

| Token | Value | Use it for |
|---|---|---|
| `--cat-ink-strong` | `#ffffff` | The strongest text — titles, the wordmark's name |
| `--cat-ink` | `#e8e9ec` | Ordinary readable text |
| `--cat-body` | `#c9cad0` | Body copy, `.cat-app`'s default colour |
| `--cat-dim` | `#9a9ba1` | Secondary text, eyebrow labels, key/value keys |
| `--cat-quiet` | `#8a8c94` | Quiet labels |
| `--cat-faint` | `#5c5e65` | Decoration and disabled text only — never a word that has to be read |

### Signal

| Token | Value | Use it for |
|---|---|---|
| `--cat-signal` | `#e94560` | The one accent colour: primary action, progress, fault |
| `--cat-signal-lt` | `#ff5d76` | Hover/active state of the signal colour, and `--cat-bad` |
| `--cat-signal-dk` | `#8e2739` | A darker step of the signal colour |
| `--cat-signal-ink` | `#ff92a3` | Text on a signal tint |
| `--cat-signal-tint` | `rgba(233, 69, 96, .14)` | A signal-tinted fill (`.cat-btn--primary`, `.cat-pill`) |
| `--cat-signal-tint-2` | `rgba(233, 69, 96, .22)` | A stronger signal-tinted fill, e.g. hover on `.cat-btn--primary` |
| `--cat-signal-line` | `rgba(233, 69, 96, .55)` | A signal-coloured border |
| `--cat-on-signal` | `#ffffff` | Text/icon colour on a solid-crimson fill |

### Status

Fault is the signal itself, at its brightest: an instrument has one red. `ok`/`warn`/`info` are the
only other hues on any screen.

| Token | Value | Use it for |
|---|---|---|
| `--cat-ok` | `#58c98b` | Success / healthy status |
| `--cat-warn` | `#ffbf63` | Warning status |
| `--cat-bad` | `var(--cat-signal-lt)` | Fault status — the signal colour at its brightest |
| `--cat-info` | `#7fa8c9` | Informational status |
| `--cat-ok-tint` | `rgba(88, 201, 139, .14)` | Success fill |
| `--cat-warn-tint` | `rgba(255, 191, 99, .14)` | Warning fill |
| `--cat-bad-tint` | `rgba(255, 93, 118, .14)` | Fault fill |
| `--cat-info-tint` | `rgba(127, 168, 201, .14)` | Info fill |

### Type

| Token | Value | Use it for |
|---|---|---|
| `--cat-sans` | `"Segoe UI Variable Display", "Segoe UI", ui-sans-serif, system-ui, sans-serif` | Display type: titles, the wordmark, watermark |
| `--cat-sans-text` | `"Segoe UI Variable Text", "Segoe UI", ui-sans-serif, system-ui, sans-serif` | Body text, `.cat-app`'s default font |
| `--cat-mono` | `"Cascadia Code", "Cascadia Mono", ui-monospace, Consolas, monospace` | Mono: eyebrows, chips, pills, key/value values, readouts |
| `--cat-fs-11` | `11px` | Eyebrow labels |
| `--cat-fs-12` | `12px` | |
| `--cat-fs-13` | `13px` | Body small, mono values |
| `--cat-fs-15` | `15px` | Body |
| `--cat-fs-18` | `18px` | |
| `--cat-fs-22` | `22px` | Section titles |
| `--cat-fs-28` | `28px` | |
| `--cat-fs-40` | `40px` | One number per screen, at most |
| `--cat-w-thin` | `200` | The wordmark's "Frc" and "2.0", nothing else |
| `--cat-w-normal` | `400` | |
| `--cat-w-medium` | `500` | |
| `--cat-w-semi` | `600` | |
| `--cat-w-bold` | `700` | |
| `--cat-track-eyebrow` | `.16em` | Letter-spacing for eyebrows (1.6 at 11px, as drawn) |
| `--cat-track-tight` | `-.02em` | Letter-spacing for display sizes only |

### Shape

Concentric: an inner radius is its parent's radius less the padding. Catalyst Console re-points
these to 2px; everything else follows the drawings.

| Token | Value | Use it for |
|---|---|---|
| `--cat-r-window` | `20px` | The window's own corner |
| `--cat-r-card` | `16px` | `.cat-card`, `.cat-glass` |
| `--cat-r-chip` | `10px` | A square-ish chip, a tile's inner element |
| `--cat-r-field` | `10px` | `.cat-field` |
| `--cat-r-inner` | `8px` | An inner element inset from a card |
| `--cat-r-pill` | `999px` | `.cat-btn`, `.cat-chip`, `.cat-pill`, the `ok`/`info` status dots |
| `--cat-lift` | `inset 0 1px 0 rgba(255,255,255,.045)` | The top edge of a surface, catching the one light |
| `--cat-lift-strong` | `inset 0 1px 0 rgba(255,255,255,.07)` | The same, on something raised or hovered |

Two notes on shape, because the first version of this file had both wrong.

**The scale is Bezel's, brought down to a window.** Bezel's tile is radius 32 with 24 of padding on a
720-wide panel. The first cut here was 12 and 16, which is what made the first screens read as boxes
ruled on a page rather than as surfaces; and Catalyst Console squared everything to 2px on the theory
that an instrument has hard corners, which beside the app read as unfinished rather than precise.
What makes the Console an instrument is its near-black ground, its hairlines, the absence of shadow
and one accent — none of which is a corner.

**Controls are capsules.** `.cat-btn` and `.cat-chip` take `--cat-r-pill`, because the one
control-shaped element in the launch material is the version capsule and everything else in the
family should agree with it.

**There are no drop shadows.** Depth comes from tone: a surface steps up, catches the light on its
top edge with `--cat-lift`, and separates with a hairline. A shadow on a near-black ground is a
smudge, and the one exception is glass, which is floating and says so.
| `--cat-space-1` | `4px` | |
| `--cat-space-2` | `8px` | |
| `--cat-space-3` | `12px` | |
| `--cat-space-4` | `16px` | |
| `--cat-space-5` | `24px` | |
| `--cat-space-6` | `32px` | |
| `--cat-space-7` | `48px` | |

### Motion

Detent springs, sampled to CSS. Durations are the full settle time; the visible part is about half.
See the [motion table](#motion) below for what each role is for.

| Token | Value |
|---|---|
| `--cat-dur-hold` | `.218s` |
| `--cat-ease-hold` | `linear(0, 0.0057 1.2%, 0.0229 2.5%, 0.0519 3.9%, 0.0949 5.5%, 0.1867 8.3%, 0.41 14.4%, 0.5139 17.4%, 0.6106 20.5%, 0.6946 23.6%, 0.7655 26.7%, 0.8239 29.8%, 0.8735 33.1%, 0.914 36.6%, 0.9462 40.4%, 0.9707 44.6%, 0.9876 49.2%, 0.9985 54.6%, 1.005 66.5%, 1)` |
| `--cat-dur-release` | `.729s` |
| `--cat-ease-release` | `linear(0, 0.0057 1.2%, 0.023 2.5%, 0.0521 3.9%, 0.0953 5.5%, 0.1362 6.8%, 0.1878 8.3%, 0.4129 14.4%, 0.5178 17.4%, 0.6153 20.5%, 0.7 23.6%, 0.7713 26.7%, 0.8298 29.8%, 0.8794 33.1%, 0.9186 36.5%, 0.9499 40.2%, 0.9738 44.3%, 0.9905 48.9%, 1.0008 54.2%, 1.0063 65.4%, 1)` |
| `--cat-dur-smooth` | `.761s` |
| `--cat-ease-smooth` | `linear(0, 0.0052 1.1%, 0.0193 2.2%, 0.045 3.5%, 0.0809 4.9%, 0.1585 7.4%, 0.3667 13.4%, 0.4649 16.4%, 0.559 19.6%, 0.6407 22.8%, 0.712 26.1%, 0.7726 29.5%, 0.823 33%, 0.8662 36.8%, 0.904 41.2%, 0.9337 46%, 0.9567 51.4%, 0.9739 57.7%, 0.9851 64.6%, 0.9926 73%, 1)` |
| `--cat-dur-settle` | `.609s` |
| `--cat-ease-settle` | `var(--cat-ease-smooth)` — same critically damped shape |
| `--cat-dur-effect` | `.239s` |
| `--cat-ease-effect` | `var(--cat-ease-smooth)` |
| `--cat-dur-layer` | `.155s` |
| `--cat-ease-layer` | `var(--cat-ease-smooth)` |

### Glass

Floating layers only: dialogs, the command palette, a toast. A thin edge line, no bevel, no
spotlight. Values from Bezel's glass table.

| Token | Value | Use it for |
|---|---|---|
| `--cat-glass-blur` | `10px` | Backdrop blur radius |
| `--cat-glass-fill` | `color-mix(in srgb, var(--cat-ground) 78%, transparent)` | Glass fill |
| `--cat-glass-edge` | `rgba(255, 255, 255, .14)` | Glass border |
| `--cat-glass-shadow` | `0 24px 60px rgba(0, 0, 0, .55)` | Glass drop shadow |

## Contrast floors

These are the measured pairs `identity.css` checks its ink tokens against — reported here exactly
as the file states them, not recalculated:

- Body on ground: **11.8:1**
- Quiet labels: **6.6:1** on a surface, **5.8:1** on the ground
- White on solid crimson: **3.83:1** — large text and icons only, never a sentence of body text.
  This is why a filled crimson button (`.cat-btn--loud`) carries one short label and nothing
  smaller.

The print material's body grey (`#70727a`) is 3.7:1. The UI lifts that a step to `--cat-body`
(`#c9cad0`), which is why the on-screen page and the printed poster read differently even though
they share a palette.

## Primitives

Every class `identity.css` defines outside `:root`, with a minimal usage example.

### `.cat-eyebrow`

A quiet mono label that names a region. Uppercase is the markup's job, not `text-transform`, so a
screen reader reads words rather than letters.

```html
<span class="cat-eyebrow">SYSTEM STATUS</span>
```

### `.cat-rule`

The hairline under a wordmark: crimson at the left, gone by the right. `.cat-rule--plain` is the
neutral variant, in `--cat-line`.

```html
<hr class="cat-rule">
<hr class="cat-rule cat-rule--plain">
```

### `.cat-card`

The base surface for a panel, with a header, a title and a border.

```html
<div class="cat-card">
  <div class="cat-card__head">
    <span class="cat-card__title">Drivetrain</span>
  </div>
  <p>Body content.</p>
</div>
```

### `.cat-rail`

A list whose items are marked by a crimson rail, as in the release card.

```html
<div class="cat-rail">
  <div class="cat-rail__item">
    <span class="cat-rail__title">Commands v3</span>
    <p class="cat-rail__body">Five decorators renamed.</p>
  </div>
</div>
```

### `.cat-btn`

Buttons. Press answers on pointer-down with a state layer, per Bezel. The base class alone is a
neutral button; three modifiers change its weight.

```html
<button class="cat-btn">Cancel</button>

<!-- .cat-btn--primary: a tinted accent button -->
<button class="cat-btn cat-btn--primary">Apply</button>

<!-- .cat-btn--loud: the one loud button on a screen, solid crimson, short label only -->
<button class="cat-btn cat-btn--loud">Deploy</button>

<!-- .cat-btn--ghost: no fill, no border, for a low-emphasis action -->
<button class="cat-btn cat-btn--ghost">Dismiss</button>
```

### `.cat-pill`

The version badge, exactly as drawn: a capsule with a lit dot.

```html
<span class="cat-pill">v2.0.0-alpha.3</span>
```

### `.cat-chip`

A mono, bordered chip. `.cat-chip--on` is the active/selected state, in the signal colour.

```html
<span class="cat-chip">can_s0</span>
<span class="cat-chip cat-chip--on">ENABLED</span>
```

### `.cat-field`

A text input.

```html
<input class="cat-field" placeholder="CAN ID">
```

### `.cat-tabs`

A tab strip. The selected tab gets a crimson underline via `aria-selected`.

```html
<div class="cat-tabs">
  <button class="cat-tab" aria-selected="true">Overview</button>
  <button class="cat-tab" aria-selected="false">Logs</button>
</div>
```

### `.cat-kv`

Key and value, the way an instrument prints them: label quiet and left, value mono and right,
digits in columns.

```html
<div class="cat-kv">
  <span class="cat-kv__key">Bus voltage</span>
  <span class="cat-kv__value">12.4 V</span>
</div>
```

### `.cat-readout`

A large mono number — one per screen, at most.

```html
<div class="cat-readout">3850</div>
```

### `.cat-dot`

A status dot. Shape carries the meaning as well as colour, so a greyscale screenshot or a
colour-blind reader still tells the states apart: `ok`/`info` are round, `warn` is a rotated
square (a diamond), `bad` is a square.

```html
<span class="cat-dot cat-dot--ok"></span>
<span class="cat-dot cat-dot--warn"></span>
<span class="cat-dot cat-dot--bad"></span>
<span class="cat-dot cat-dot--info"></span>
```

### `.cat-wordmark`

Thin grey prefix, bold white name, thin crimson version.

```html
<span class="cat-wordmark">
  <span class="cat-wordmark__prefix">Frc</span
  ><span class="cat-wordmark__name">Catalyst</span
  ><span class="cat-wordmark__version">2.0</span>
</span>
```

### `.cat-watermark`

The oversized figure behind a page. It is never read, so keep it out of the accessibility tree
(`aria-hidden="true"`).

```html
<span class="cat-watermark" aria-hidden="true">2.0</span>
```

### `.cat-glass`

Glass, for floating layers only — a dialog, the command palette, a toast.

```html
<div class="cat-glass">
  <p>Toast content.</p>
</div>
```

## Motion

The roles `identity.css` and `motion.js` both name. CSS carries each role as a `--cat-dur-*` /
`--cat-ease-*` pair; `motion.js`'s `ROLE` map carries the same roles as SwiftUI-style
response/damping specs for the two things CSS can't do — following a gesture's release velocity,
and answering a press at the point it was pressed.

| Role | What it's for | Detent preset (response / damping) | Duration |
|---|---|---|---|
| `hold` | A press being held, a value under the pointer | response `0.15`, damping `0.86` | `.218s` |
| `release` | Something arriving: a panel, a menu, a toast (snappy: bounce 0.15) | response `0.5 / 1.36`, damping `0.85` | `.729s` |
| `smooth` | Something leaving, which never bounces | response `0.5 / 1.36`, damping `1` | `.761s` |
| `settle` | A value travelling to where it was set | response `0.4`, damping `1` | `.609s` |
| `effect` | Colour, opacity, glass | same critically damped shape as `smooth` | `.239s` |
| `layer` | The state layer under a pointer | same critically damped shape as `smooth` | `.155s` |
| `detent` | Landing on a detent (`motion.js`'s `ROLE.detent`, no CSS pair) | response `0.4`, damping `0.8` | — |

`motion.js` also carries the gesture constants a spring alone can't express: `hysteresis` (10px
before a touch is a drag, iOS), `sliderHysteresis` (3px — a slider claims the touch at once),
`decelerationRate` (`0.998`, UIScrollView's coast), `projectionRate` (`0.99`, WWDC18's "where will
it stop" for snap targets), `rubberBand` (`0.55`, iOS past an edge) and `maxHandoff` (5000 px/s,
the velocity cap handed to a spring).

## Adoption

To bring the identity into a new Catalyst surface:

1. Copy `identity.css` and `motion.js` in verbatim. They are meant to be copied, not imported from
   a shared package — CatalystApp and Catalyst Console each keep their own copy.
2. Link `identity.css` first, before any of the product's own stylesheets, so its tokens are
   available when the product's own rules resolve.
3. Alias any of the product's existing token names to the `--cat-*` tokens rather than
   hand-porting values, so the product picks up future identity changes by re-copying the file.
4. Override only the shape tokens (`--cat-r-*`) when the product needs different geometry — leave
   colour, type and motion alone.

Catalyst Console is the real example: it is an instrument panel, so `src/styles.css` re-points
`--cat-r-window`, `--cat-r-card`, `--cat-r-chip`, `--cat-r-field` and `--cat-r-inner` to `2px` and
changes nothing else, and aliases its own `--r-sm` / `--r` / `--r-lg` tokens to the `--cat-r-*`
ones.

## Rules that are easy to break

- **Glass only on floating layers, never glass-on-glass.** `.cat-glass` is for a dialog, the
  command palette or a toast — not for a card sitting on the ground, and never for one glass
  surface stacked on another.
- **The signal colour never appears on small text.** White on solid crimson is 3.83:1, which holds
  up for a large label or an icon and nothing smaller — see [Contrast floors](#contrast-floors).
- **Status colour is never the only signal.** Every status level also gets a glyph or a shape — the
  `.cat-dot` modifiers are round, diamond and square, not just three colours of the same circle —
  so a screenshot in greyscale or a colour-blind reader still tells them apart.
- **Nothing that's redrawn every frame gets a CSS transition.** A `transition` animates a property
  change once; a value that updates every frame (a gauge, a live readout, a dragged element) is
  driven by `motion.js`'s `spring()` instead, because a transition retargeted every frame never
  reaches its target and looks like lag.
