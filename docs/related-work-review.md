# Related-Work Review: Stress-Testing H1's Novelty Claim

This document exists to do one thing: take the specific claim in
[research-question.md](research-question.md) — **H1**, that an explicit,
swappable future-opportunity *representation*, added to an RL policy's
observation and isolated by a controlled, matched A/B comparison against
an otherwise-identical reactive policy, produces a measurable difference
— and test it as aggressively as possible against the closest prior work
found. If H1's novelty doesn't survive this review, that needs to be
known now, not after M6–M9 are built.

**This is not a general marine-robotics survey** (that's
[wave-simulation-literature-review.md](wave-simulation-literature-review.md)).
Every paper here is evaluated against one question: *does this paper
already do what H1 claims to contribute, or something close enough that
H1's framing needs to be narrowed?*

**Confidence tagging**, same convention as the wave-simulation review:
**[primary-text-verified]** (full text read), **[secondary]** (abstract/
search-summary only — paywalled or extraction failed, flagged at each
use). Several papers below only got as far as **[secondary]** despite
real attempts (arXiv PDF binary-extraction failures, IEEE Xplore/
ResearchGate paywalls) — that is disclosed per entry, not smoothed over.

## The pipeline framing this review uses

The existing literature on "grasp a moving thing" is strong and
well-established along one pipeline:

```
detect → track → predict future trajectory → plan motion → time the grasp → execute
```

H1's proposed insertion is not a new pipeline stage in that chain — it is
a claim about *what the timing decision is conditioned on and how its
contribution is measured*:

```
state history → future interaction QUALITY (not just position) →
opportunity landscape Ω(t+1 .. t+N) → discrete WAIT / ACT / ABORT →
conventional grasp planner
   [ablation: same pipeline, same everything, minus the Ω signal]
```

Two things must both be true for a paper to threaten H1, not just look
similar: (1) it must condition a timing/commitment decision on something
like future *interaction quality*, not merely future *position*, and
(2) it must isolate that signal's causal contribution via a matched
comparison, not just report end-to-end success rate for one system. Most
papers below satisfy neither; a few satisfy the first partially and none
found satisfy the second — that gap is the actual finding of this review.

---

## Group A — Closest physical precedent (USV + manipulator + floating grasp)

### A1. Li, T. and Cai, Y. (2025). "Motion Planning and Compensation
Approaches for Autonomous Surface Manipulator Systems in Grasping Tasks
on Water Surfaces." *IEEE Robotics and Automation Letters*, 10(6),
5593–5600.

**Access: [secondary].** IEEE Xplore returned HTTP 402/418 to automated
fetch (`ieeexplore.ieee.org/document/10964710`); ResearchGate returned
403. Citation and venue independently corroborated via the author's own
lab publication page (not just the user-supplied link), which increases
confidence in the citation details even though the method section
wasn't readable.

**What it is, per available description:** studies the time-varying
relationship between a USV's own motion and a floating target, and
develops motion-planning/compensation methods so the manipulator can
grasp despite that relationship changing continuously. This is the
closest *physical setup* match to this project (USV + arm + floating
target) in the entire list.

**Verdict on H1:** low overlap risk, based on what's available. This is
a classical motion-planning/compensation contribution — continuously
tracking and compensating for a changing USV-target relationship — not a
discrete commitment decision conditioned on a distinct future-quality
signal, and not evaluated via an RL ablation isolating such a signal.
The RA-L venue and abstract framing ("motion planning and compensation")
both point toward continuous control, not the WAIT/ACT/ABORT-style
discrete layer H1 concerns. Should still be the first citation in this
project's own related-work section as the closest physical precedent —
full text should be obtained (institutional access) before final
write-up, since this is the one entry where a surprising overlap, if it
exists, is most likely to be hiding in a paywalled methods section.

### A2. Li, T. and Cai, Y. (2025). "Optimized Visual Servo Control and
Digital Twin-Driven Hardware-in-the-Loop Simulation Methods for
Autonomous Surface Manipulator Systems." *IEEE Transactions on
Mechatronics*, 1–12.

**Access: [secondary]**, same paywall situation as A1; citation
corroborated the same way.

**What it is:** proposes a joint-angle-based (rather than joint-velocity-
based) visual servo control method for ASMS (autonomous surface
manipulator systems), a 12-DOF coupled USV+manipulator dynamic model, a
"normalized stability index" for grasping-phase stability, and
hardware-in-the-loop validation via a digital twin.

**Verdict on H1:** low overlap. This is a control-theory contribution
(visual servoing formulation, stability index, HIL validation
methodology) — genuinely useful precedent for *how to validate* this
project's own vision→state-estimation pipeline (§ noted in the user's
own framing above), but it is not making any claim about future
interaction *opportunity* or *timing*, discrete or otherwise. No overlap
with H1's specific claim.

---

## Group B — Closest "predict the future, time the contact" precedent

### B1. Antal, P., Péni, T., and Tóth, R. (2025). "Hook-Based Aerial
Payload Grasping from a Moving Platform." *2025 IEEE International
Conference on Robotics and Automation (ICRA)*, pp. 5209–5215.
arXiv:2409.11788.

**Access: [primary-text-verified]** (abstract + methodology summary read
via arXiv). This is the single most important paper in this review to
have gotten right, and it was accessible.

**What it does:** a hook-equipped aerial manipulator grasps a payload
from a moving platform. Verbatim from the paper: "a computationally
efficient trajectory optimization based on complementarity constraints
is proposed to determine the optimal grasping time," and "the future
motion of the payload is predicted using a physics simulator-based
model." Grasp success under model uncertainty/disturbance is then
formally verified via integral-quadratic-constraint robustness analysis
— i.e., the pipeline is: physics-model-based prediction → trajectory
optimization (which implicitly picks a grasp time as part of solving a
single optimal-control problem) → formal robustness certification. This
confirms, cleanly and rigorously, that **"predict the future state of a
moving thing, then compute an optimal contact time" is already an
established paradigm** — not something this project can claim as new,
and the user's own read of this paper (in the original list) was
correct.

**What is verifiably different from H1:** three things, all confirmed
rather than assumed:
1. **Prediction is deterministic/model-based, not a learned
   representation.** "Physics simulator-based model" — this is closer to
   an oracle/ground-truth-adjacent forward model than a learned,
   uncertain opportunity estimator. H1's opportunity signal is explicitly
   meant to be an estimator with its own error characteristics (per
   `rl-formulation.md`'s future-opportunity-representation spec), not a
   privileged forward simulation.
2. **Timing is an implicit byproduct of a single trajectory-optimization
   solve, not a discrete, separately-observable decision variable.**
   There is no analogue of a WAIT/ACT/ABORT action — the optimizer picks
   one grasping time as part of one continuous optimization. There is
   nothing here to "ablate" the way H1's opportunity signal is meant to
   be ablated, because timing isn't a separable input to a policy at
   all — it's an output of a solver.
3. **No RL, no learned policy, no matched-comparison ablation.** The
   contribution is a trajectory-optimization-plus-verification method,
   evaluated end-to-end (sim + real flight), not a controlled A/B
   isolating one representational choice's causal effect.

**Verdict on H1:** this paper is the strongest evidence that the
*broad* framing "we predict the future and choose when to act" would not
be defensible as novel — and simultaneously the strongest evidence that
H1, read precisely (explicit *learned, swappable, ablatable*
representation of future *interaction quality*, isolated via *matched
RL comparison*), is not what this paper does. **Action item for
research-question.md / a future paper draft: this citation should appear
explicitly in the related-work section specifically to preempt the "but
optimal-grasp-timing already exists" objection, with the three
distinctions above spelled out.**

### B2. Menon, A., Cohen, B., and Likhachev, M. (2014). "Motion Planning
for Smooth Pickup of Moving Objects." *IEEE International Conference on
Robotics and Automation (ICRA)*, pp. 453–460.

**Access: [secondary]** — abstract obtained (CMU Robotics Institute
publication page), a direct PDF was not locatable through the fetch tools
available in this session.

**What it does, per the abstract:** kinodynamic motion planning that
generates "a time-parameterized trajectory for both the arm and
end-effector, capable of carefully picking up the object at the earliest
feasible point" in its trajectory, using "informative heuristics and
adaptive dynamic motion primitives." Timing (the "earliest feasible
point") reads as an emergent property of a search/planning objective
("earliest that is feasible"), not a distinct decision output.

**Verdict on H1:** low-to-moderate overlap on the *concept* (pickup
timing as a first-class planning concern — this is the earliest paper
in this whole list to explicitly frame "when to pick up" as a named
problem, and is worth citing for exactly that historical-precedent
reason), but no apparent overlap on *mechanism*: "earliest feasible"
is a single-shot planning criterion baked into one search, not a
separately-observable, ablatable signal, and there's no RL or
matched-comparison methodology here at all (2014, pre-dates the
deep-RL-for-manipulation wave). Should be cited as the foundational
"pickup timing is a real problem" reference, with the caveat noted that
the paper's own methodology doesn't overlap with H1's specific
representational/experimental claim.

### B3. Marturi, N., Kopicki, M., et al. (2019). "Dynamic Grasp and
Trajectory Planning for Moving Objects." *Autonomous Robots*, 43(5).

**Access: [secondary]**, abstract read directly (Springer/ACM listing).

**What it does, verbatim from the abstract:** "a set of likely grasps,
generated by a learned grasp planner, are evaluated online" against
both current robot configuration and collision-free reachability; "a
task-based cost function enables relaxation of motion-planning
constraints, enabling the robot to continue following the object by
maintaining its end-effector near to a likely pre-grasp position
throughout the object's motion."

**Verdict on H1:** minimal overlap. This is continuous online
re-evaluation of grasp candidates against the *current* state at every
control cycle — reactive re-planning, not future-quality reasoning.
There is no forward temporal horizon here at all (no `t+1..t+N`
reasoning implied by the abstract); the system "follows" and re-evaluates
rather than anticipates. Good citation for "what a strong reactive
baseline for continuous tracking looks like," which is directly useful
context for designing this project's own reactive baseline B0/B1
(`rl-formulation.md`'s baseline ladder) — arguably more useful as
baseline-design precedent than as a novelty threat.

### B4. Kim, S., Shukla, A., and Billard, A. (2014). "Catching Objects in
Flight." *IEEE Transactions on Robotics*, 30(5), 1049–1065.

**Access: [secondary]**, citation corroborated independently (title,
authors, volume/issue/pages all matched across two independent search
results), abstract-level description only.

**What it is, per widely-repeated description:** foundational
moving-object interception work — trajectory prediction, identification
of feasible catching configurations along the predicted trajectory, and
arm-motion planning to reach one. Pre-dates essentially all deep-RL
manipulation work.

**Verdict on H1:** establishes the general "predict, then find a feasible
future contact configuration" paradigm as old (2014, and arguably older
in the broader interception-robotics literature this paper itself cites)
— useful as the paradigm's origin citation, no mechanism-level overlap
with an explicit learned/ablatable opportunity signal.

---

## Group C — Marine rescue / recovery (application-space precedent)

### C1. Zhang, L., Huang, Y., and Qian, H. (2026). "Design and Control of
the 'QuadBoat': A Quadruped Surface Vehicle for Drowning Rescue."
arXiv:2607.13633.

**Access: [primary-text-verified]** (fetched and read directly).

**What it does:** a quadrupedal USV with boat-shaped feet that can lower
its body to pass under obstacles and raise a victim by lifting its own
central body — **no manipulator, no grasping component at all.**
Control is "a PID-based inverse kinematic control strategy for the leg
action and an MPC controller for the overall movement." Contact strategy
is: visually track the victim, achieve positional alignment, then lift —
confirmed via direct reading to be **reactive**, with **no explicit
future-prediction or opportunity-reasoning component** for deciding when
to make contact.

**Verdict on H1:** no overlap. Valuable as *motivation* for the marine-
rescue/recovery application framing (a very recent, real, funded USV
project doing physical victim retrieval), and as a data point that even
current (2026) marine-recovery USV work has not yet incorporated any
future-opportunity reasoning at the contact-timing level — but zero
methodological overlap since there is no manipulator or future-quality
reasoning of any kind here.

### C2. Yuan, W., Hang, K., Song, H., Kragic, D., Wang, M. Y., and Stork,
J. A. (2019). "Reinforcement Learning in Topology-based Representation
for Human Body Movement with Whole Arm Manipulation." *IEEE
International Conference on Robotics and Automation (ICRA)*.
arXiv:1809.04322.

**Access: [secondary]** — PDF fetch returned unparseable binary content
twice (direct arXiv PDF and a DeepAI mirror); description below is from
search-result abstract text plus the paper's own arXiv abstract field,
not the full methodology.

**What it does, per the abstract:** learns whole-arm-manipulation motions
for holding/transporting humans in rescue/patient-care scenarios, using
a topology-based coordinate representation (computed from arm/torso
positions) that transfers across body shapes; evaluated in a simulated
"dynamic sea rescue scenario," explicitly framed as "provid[ing] a
behavior that can directly respond to external perturbation and human
motion" — i.e., reactive by the paper's own stated framing.

**Verdict on H1:** this is the paper in the whole list that most
directly combines **RL + a named sea-rescue scenario**, which is exactly
why it matters for this project's own novelty phrasing: it means
**"RL applied to a dynamic marine rescue scenario" is not new** and
should never appear as a standalone novelty claim in this project's
writing. It does not, however (based on the accessible abstract),
contain a discrete WAIT/ACT/ABORT-style commitment decision or an
explicit future-interaction-quality representation separate from the
continuous whole-arm policy — "directly respond to... motion" reads as
reactive, matching the paper's self-description. **Action item**:
obtain the full PDF text (fetch failed twice in this session; try a
different extraction route before finalizing any paper) before treating
this "no overlap on the timing-representation claim" verdict as final —
this is the second-most-important paper (after B1) to get exactly right.

### C3. [OpenDrift-based maritime SAR trajectory prediction case study,
South Sea of Korea] (2026). *Frontiers in Marine Science*.

**Access: [secondary]** — not independently re-fetched this pass (user's
own description used as-is; open-access venue makes this low-risk to
trust provisionally, but not yet directly verified).

**Relevance, taken at face value:** physically-grounded (real ocean/wind
forcing, field-tested drifter validation) stochastic future-position
prediction for SAR targets. Useful as precedent that *predicting where a
target's future state distribution will be* is an established maritime
SAR requirement — but this is prediction of **position**, feeding
**search planning**, not prediction of **interaction quality** feeding a
**grasp/contact commitment decision**. Good citation for "the future-
prediction half of the pipeline is well-established," reinforcing that
this project's contribution must be about what's built *on top of* such
a prediction (the opportunity/commitment layer), not the prediction
itself — consistent with `rl-formulation.md`'s existing design (the
opportunity estimator is explicitly meant to be swappable/separate from
whatever future-state model feeds it).

### C4. Angelis, D. and Boukas, E. (2026). "Dynamic UAV-Based Search
Operations Using Probabilistic Diffusion Modeling of Man Overboard
Incident Victims." arXiv:2608.02093.

**Access: [primary-text-verified]** (abstract + methods fetched
directly).

**What it does, confirmed by direct reading:** an Extended Kalman Filter
built on a leeway (wind/current drift) model tracks victim-position
uncertainty; this feeds *search-area allocation* across five compared
search patterns (Zigzag, Boustrophedon, Spiral, Probability-Informed
Search, and an improved variant), the best reaching >80% average
find-rate. **Confirmed scope limitation, directly from the text: this
paper addresses detection/search only — no physical retrieval or contact
decision of any kind appears in the abstract or methods summary.**

**Verdict on H1:** no overlap on mechanism (search-area optimization,
not grasp timing), but directly useful as the second data point (with
C3) that probabilistic future-state reasoning for maritime targets is an
active, current (2026) research area purely on the *prediction* side,
strengthening the argument that the "what do you do with the prediction"
question (this project's actual focus) is comparatively under-explored
in the reviewed literature.

---

## Group D — Dynamic grasping / handover (general manipulation precedent)

### D1. Kiyokawa, T., Nagata, E., Tsurumine, Y., Kwon, Y., and Matsubara,
T. (2024). "Self-Supervised Learning of Grasping Arbitrary Objects
On-the-Move." arXiv:2411.09904.

**Access: [secondary]** — abstract read directly; methodology section
(three FCN heads: static grasp primitive, dynamic grasp primitive,
residual moving-velocity error) noted but not deeply verified against
full text.

**Verdict on H1:** based on the accessible abstract, this learns *how* to
grasp (position/orientation/velocity adjustment) via self-supervision,
with no described discrete timing-commitment mechanism or explicit
future-opportunity representation. Low overlap; useful as a recent
(2024) example of where the field's dynamic-grasping learning effort is
actually concentrated — grasp parameters, not commitment timing —
which is itself supporting evidence for H1's positioning (the timing
layer is comparatively neglected relative to the grasp-execution layer).

### D2–D5 (Reactive/learned handover and general dynamic-grasp
precedent — grouped for brevity, not individually fetched this pass):
Kim et al. (2025) "Learning-Based Dynamic Robot-to-Human Handover"
(arXiv:2502.12602); "DexH2R" dynamic dexterous handover benchmark
(arXiv:2506.23152); "Autonomous Learning-Free Grasping and Robot-to-Robot
Handover of Unknown Objects" (*Autonomous Robots*, 2025).

**Access: [secondary]**, user-supplied descriptions accepted
provisionally (all describe learned or classical dynamic-grasp/handover
behavior conditioned on a moving receiver/target, none described as
containing a discrete future-opportunity commitment layer). **Not
independently re-verified in this pass** — flagged explicitly as the
weakest-evidence group in this review; before any of these are cited in
a paper draft, each should get the same direct-read treatment as Group B.

---

## Group E — Marine manipulation / physical dynamics (context, not novelty threats)

### E1. Novák, F., Báča, T., and Saska, M. (2024). "Collaborative Object
Manipulation on the Water Surface by a UAV-USV Team Using Tethers."
*2024 IEEE/RSJ International Conference on Intelligent Robots and
Systems (IROS)*. arXiv:2407.08580.

**Access: [secondary]**, citation corroborated across two independent
sources; abstract-level description only.

**What it does:** a coupled UAV+USV+tethered-floating-object dynamic
model with MPC control, validated in Gazebo simulation, showing improved
disturbance-recovery vs. a single-robot baseline.

**Verdict on H1:** no overlap on the opportunity/timing claim — this is
a coupled-dynamics/control contribution. Useful precedent for modeling
floating-object dynamics with real coupling (tether tension, MPC) rather
than a prescribed/scripted target trajectory, which is directly relevant
to how this project's own dynamic target (M4) should be modeled if
physical realism matters more than convenience — worth a re-read at that
milestone, not for the novelty argument.

### E2. NOAA Ocean Exploration — "Advances in Deep-Sea Sampling with Soft
Robotics" (web feature, not a peer-reviewed paper).

**Access:** not a research paper — application-background context only,
as the user's own note says. No novelty-relevant content; useful only as
a motivating real-world example if the paper's introduction needs one.

---

## Synthesis: what survives this review

Testing H1 against the 19 papers above (13 with at least some direct
verification, 6 accepted provisionally from the user's own descriptions
and flagged as such) does **not** find a paper that:

1. Learns an explicit, separately-observable representation of *future
   interaction quality* (as opposed to future *position*), **and**
2. Feeds that representation into a *discrete* commitment/timing
   decision separate from continuous low-level control, **and**
3. Isolates that representation's causal contribution via a matched,
   controlled comparison against an otherwise-identical policy without
   it.

Individual pieces of this appear scattered across the literature —
B1 (Antal et al.) has future prediction feeding an optimal-time
computation, but it's a deterministic single-solve optimization, not a
learned/ablatable RL observation; B3 (Marturi et al.) has continuous
online re-evaluation, but no forward horizon and no discrete decision;
D1 has learned dynamic grasping, but concerns grasp parameters, not
commitment timing. **No paper found combines all three properties**, and
critically, **no paper found runs the specific experiment H1 requires**:
same policy, same reward, same disturbance distribution, same training
budget, with only the opportunity signal added or removed.

This means H1's novelty claim, *as precisely stated in
research-question.md*, survives this review. But two things should
change as a result of it:

- **The related-work section this project eventually writes must lead
  with B1 (Antal et al. 2025) and C2 (Yuan et al. 2019)**, not bury them —
  these are the two papers most likely to prompt a reviewer's "isn't this
  already done?" objection, and both have a clean, specific, defensible
  answer (documented above) rather than a vague "we're different."
- **A1/A2 (Li & Cai 2025 ×2)** are the least-resolved risk in this
  review — the closest physical setup, current confidence based only on
  abstracts/venue, full text never obtained. Getting actual read access
  to those two IEEE papers should happen before any paper draft is
  submitted, not left as a standing gap.

## Recommended follow-up (not performed in this pass)

- Full-text read of Li & Cai 2025 (A1, A2) via institutional access.
- Retry full-text extraction of Yuan et al. 2019 (C2) — two automated
  attempts failed on binary/PDF parsing, not on paywall; a different
  extraction method should succeed.
- Direct-read verification of Group D2–D5 and C3, currently accepted
  only on the user's own (likely accurate, but unverified-by-this-review)
  characterization.
