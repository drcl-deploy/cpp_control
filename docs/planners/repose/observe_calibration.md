# The observation block — theory

How a frame of RGB-D becomes one answer: a colour, a pose, or a rejection.
Collection procedure and the tuning commands are
[color_calibration.md](color_calibration.md); the knob schema is
[planner.md §5.5](planner.md).

## 1. The shared front

Both reads start identically, and neither is a colour test on an image:

```
  RGB + depth
      |
      |  GATES     hi >= min_value  and  (hi-lo)/hi >= min_rel_sat
      |            -> a pixel is a COLOUR, or it is background
      |
      |  CLASSIFY  nearest of 12 chromaticity refs in (r, g)
      |            -> one of 6 labels, or NONE if chroma_reject is on
      |
      |  UNPROJECT depth -> the robot's gravity-aligned base frame
      v            -> every test below is in METRES, not pixels
```

Chromaticity is `(r, g)` of `rgb / (r+g+b)`, so shading — which scales all three
channels — moves a pixel along a ray it never leaves. `min_rel_sat` is relative
for the same reason.

**The classifier has no reject class unless you give it one.** With
`chroma_reject: 0` every lit pixel in the frame becomes one of six colours,
including the floor.

## 2. Two reads, one question

The question is not "what colour is this pixel". It is **what counts as one
thing**.

### `blobs` — a colour is an object

```
  labelled pixels
     ├─ red    → top slab → min-area rect → up_dot? big? vis? → survivor  z=0.41
     ├─ orange → top slab → min-area rect → up_dot? big? vis? → survivor  z=0.44 ◄ wins
     ├─ green  → too few pixels
     ├─ yellow → top slab → min-area rect → up_dot? big? vis? → survivor  z=0.43
     ├─ blue   → up_dot fails: side_face
     └─ pink   → too few pixels
                                        survivors compete → HIGHEST wins
```

Six independent extractions. The height tie-break is not arbitrary: a
same-coloured floor touching the cube is ONE connected region and the floor is
the *larger* half, so you cannot pick by size — you pick by height.

### `mask` — an object is a place

```
  labelled pixels (all colours at once)
     └─ top horizontal plane   (top_pct-th height, ± band_m)
          └─ largest connected component, eroded
               └─ up_dot? big? vis?            ← gated ONCE
                    └─ count labels inside: yellow 612, orange 388 → YELLOW
```

One extraction. Find the top face as a **surface**, then ask what colour it
mostly is.

## 3. The difference, in one line

| | assumes |
|---|---|
| `blobs` | one colour = one object; competing objects are ranked by height |
| `mask` | one surface = one object; competing colours are ranked by count |

## 4. Why it matters — measured, `bags/sys1_observe`

A painted face often straddles two chromaticity cells: part of a yellow top
reads yellow, part reads orange.

| | what happens |
|---|---|
| `blobs` | the face becomes **two candidates**, both genuinely cube-shaped (`big` 1.02–1.14, `up_dot` ≈ 1.00, hundreds of px). Both pass every gate — each *is* a real piece of a real cube top — then compete on median height, where a centimetre of depth noise decides |
| `mask` | those pixels are one connected plane. Nothing to compete with; the vote settles it |

**54 of 83 wrong answers** were the true colour's blob passing every gate and
losing the height contest to a fragment of its own face. Frames carried **4.15
surviving candidates** on average; yellow and blue produced one in *all 240*.

This is why no threshold repairs `blobs`. Every gate is answering correctly —
the face really is horizontal, cube-sized, and that colour. The defect is that
one face was counted as two things that then had to fight.

## 5. Neither half works alone

| config | accuracy | false positives (60 no-cube frames) |
|---|---|---|
| `blobs`, sim palette | 53.9% | 0/60 |
| `blobs`, measured palette | 61.7% | 4/60 |
| `mask`, sim palette | 48.9% | 0/60 |
| `mask`, measured palette, `chroma_reject` 0.06, `min_rel_sat` 0.15 | **83.3%** | 0/60 |

`mask` on the wrong palette is **worse than shipping**: a modal vote is only as
good as the labels it counts, and it has no second chance from some other
fragment happening to win. Right architecture, wrong reference data.

`chroma_reject` is what makes the pair safe. With a real "none" class,
`min_rel_sat` can fall to 0.15 to catch a pale face — the deployment's green
sits at 0.28 median relative saturation, right on the old 0.22 gate — without
the floor walking into the vote.

## 6. What `mask` gives up

`blobs` can find a cube that is not the highest coloured thing in frame; each
colour gets its own slab. `mask` commits to one plane at the top of the scene,
so anything coloured *above* the cube is what it looks at instead.

Untriggered on 60 no-cube frames, and the deployment is a floor-standing cube
under a 45°-down camera — but it is a real assumption, and it is why `blobs`
stays the default rather than being deleted.

## 7. Which knobs belong to which

| knob | `blobs` | `mask` |
|---|---|---|
| `gates.*`, `palette` | ✓ | ✓ |
| `gates.chroma_reject` | ✓ (off by default) | ✓ (load-bearing) |
| `geometry.up_dot_min`, `big_max`, `min_visible*` | per candidate, ×6 | once, on the mask |
| `geometry.slab_frac` | ✓ the per-colour top slab | — |
| `mask.top_pct`, `band_m`, `erode` | — | ✓ |

## 8. The rule the belief cannot save you from

A wrong palette fails **systematically**: the same face reads the same wrong
colour in every frame of a placement. `belief.min_votes` filters *flicker*, and
a systematic misread agrees with itself — raising it only makes the planner
slower to be wrong.

Read the candidates, not the verdict:
`scripts/planners/repose/repose_check_observe.sh` is the only view that shows
the ones the gates threw away.
