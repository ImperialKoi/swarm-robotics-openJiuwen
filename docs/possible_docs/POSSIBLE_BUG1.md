# POSSIBLE BUG 1 — the swarm covers ground it never examines

**Status: diagnosed, not fixed.** No code was changed to produce any number below.
Investigated against `f9b8b8f`, `demo` scenario, `hivemind=False` (the
control condition, invariant #1). Seed 42 unless a row says otherwise.

Symptom as reported: *there are still a lot of victims left on the ground and not being
picked up*, and — across two days — **rescues will not move**. Fifteen configurations
spanning relay count, relay range, chain aim, chain length, carrier count, carry speed,
scout speed, search spread, task supply and two genuine stall-detector bugs all landed
between **40 and 45 rescues**. Several of those changes fixed real defects. None of them
mattered.

This file records why. It is the *symptom* layer; [POSSIBLE_BUG3.md](POSSIBLE_BUG3.md)
names a cause upstream of it, and §7 reconciles the two.

---

## 1. The missed casualties are not in ground nobody visited

Seed 42: 110 casualties, **68 found, 42 never found**. For every casualty, the closest any
robot ever physically came, sampled at 2 Hz for the whole mission:

| of the 42 never found | |
|---|---:|
| standing on ground the swarm mapped | 13 |
| a robot came within 8 m of them | **27** |
| **median closest approach by any robot** | **3 m** |
| minimum closest approach | **0 m** |

**The swarm walks within three metres of the people it does not find.** Coverage is not
the constraint, which rules out the entire family of fixes aimed at it.

## 2. They are looked at an order of magnitude less

Counting 5 Hz frames in which a casualty was inside some robot's 90° camera arc and within
that robot's own `sensor_radius`:

| mean frames per casualty | found | never found |
|---|---:|---:|
| within sensor range | 1003.8 | 134.1 |
| **inside the 90° FOV** | **413.0** | **35.3** |
| framed and closer than 3 m | 142.6 | **7.5** |

A found casualty gets ~413 framed looks — roughly **80 seconds** of camera time. A missed
one gets 35, about **seven seconds**. Discovery is a **dwell-time** process and the search
pattern allocates dwell very unevenly.

Splitting the 42 misses by exposure:

    never framed even once             19    <- genuine coverage failure
    framed 10+ times and still missed   21    <- looked at, not seen
    framed within 3 m at least once     16

**Half the misses were looked at repeatedly.**

## 3. Burial is the discriminator

    buried share of casualties FOUND        31%
    buried share of casualties NEVER FOUND  55%

A buried casualty renders as a small patch that only grows as debris clears
([M-7](docs/MEASUREMENTS.md)), so it needs far more looking than an unburied one — and the
search pattern gives every cell the same brief glance regardless.

## 4. The report pipeline is not the culprit

Of 4,822 reports created in one mission: 53% resolve, 20% are dismissed as phantoms, 27%
never leave `CANDIDATE` and are pruned as stale. That funnel is working as designed.

**Only 7 of the 42 missed casualties ever had a report on record within 6 m of them.** So
35 of them never produced a surviving detection at all. The loss is **upstream of the
tracker, at the camera** — not at corroboration, promotion, allocation or investigation.

## 5. What this explains

Every null result of D13 follows from it, and so does the single success:

| change | measured | why |
|---|---|---|
| faster scouts | −2.4 rescues, +4 pts explored | more ground, **less dwell per place** |
| spreading the search (cap 1) | −4.25 rescues | same: fewer looks per casualty |
| relay range 38→54 m | rescues flat, comms 83→94% | buys coverage, and coverage is not the constraint |
| relays 64→96 | +0.5 rescues | ditto |
| carriers 96→160 | +0.3 rescues | delivery was never the constraint |
| carry speed 0.7→0.85 | −4 rescues | ditto |
| chain aim, stall fixes | +1 rescue | correctness, no effect on dwell |
| **map 480×320 → 360×240** | **15.6% → 40.6% rescued** | **the same looking concentrated on 44% less ground** |

The one change that worked did not add capability. It reduced the ground over which a
fixed amount of looking had to be spread.

## 6. The metric has been measuring the wrong thing

`ground_explored_frac` counts a cell the moment a camera pixel covers it. Finding a person
there additionally requires them **inside a 90° arc, close, unoccluded, and — if buried —
very close indeed**.

**"Explored" and "examined" are different quantities and only the first is measured.**
76% explored against 64% found is the gap between them, and it is why raising coverage
stopped buying discovery some time ago. Any candidate fix should be scored on
**framed-looks-per-casualty**, not on explored fraction — otherwise it will look like an
improvement and change nothing, which has now happened fifteen times.

## 7. Relationship to POSSIBLE_BUG3

BUG3 names a cause upstream of all of the above, and it is correct. Its central claim was
re-instrumented here from scratch rather than taken on trust, and reproduces exactly:

    explore: 4432 assignments ended
       completed 2681 (60%)  dist median 88.1 m  held median 1.0 s
       ended within 5 m of goal: 320 of 4432 (7%)

`SkillExecutor._state` retires an explore task when `world.explored[iy, ix]` is true — the
**shared** map, with no mention of the robot holding the task — so the cell is revealed by
whoever is nearest and every robot assigned to it is released where it stands. The
comparison that settles it sits in the same data: `investigate` completes on *its own
holder arriving* and ends within 5 m of goal **58%** of the time against explore's **7%**.

**The two documents are one finding at two depths:**

1. Explore tasks retire on the shared map → median assignment lives **1.0 s**
2. Robots churn — assigned, released, idle, waiting for the next auction round
3. They never travel far enough to **dwell** anywhere
4. Dwell is what discovery needs: **413** framed frames for a find, **35** for a miss
5. Buried casualties need the most looking and are 55% of the misses
6. Rescues sit at 40–45 whatever else changes

BUG3 is the actionable end. This file is the evidence that it is the *only* end worth
acting on, because it rules out coverage, comms, allocation, carriers and delivery
individually and by measurement.

## 8. What is deliberately not claimed

- **No fix is proposed or made here.** Section 6 says what a fix should be *measured*
  against, not what it should be.
- **BUG2 is not reconciled with this.** It reports the same symptom and names a different
  cause (flow-field congestion and separation forces). Both may be true and compounding;
  they imply different fixes and should be settled against each other before either is
  acted on.
- The 19 casualties never framed once **are** a real coverage failure and are not
  explained by dwell. They are the far-sector problem in
  [M-53/M-54](docs/MEASUREMENTS.md), which is a comms-reach limit, still open.

## 9. Numbering

The measurements in this file are recorded in `docs/MEASUREMENTS.md` as **M-56**, and the
BUG3 verification as **M-57**. BUG2 and BUG3 both propose to land as M-56; BUG2's
flow-field numbers still need a row of their own.
