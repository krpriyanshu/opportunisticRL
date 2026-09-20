# Wave-Simulation Literature Review

This project's disturbance source is wave-driven vessel motion (see [architecture.md §5](architecture.md#5-simulator-selection)). This review establishes precisely how other marine-robotics work models ocean waves — not waves in general, but the specific equations, spectra and simulator-physics coupling each project adopted — so that this project's own choice is a deliberate decision compared against the field rather than an inherited default that was never examined.

**Method and confidence levels.** Every claim below is tagged by how it was obtained:
- **[source-verified]** — read directly from this repository's own installed code, or from a paper's/project's source code fetched and inspected line-by-line.
- **[primary-text-verified]** — read directly from the paper's own PDF text (open-access copy located and read in full).
- **[secondary]** — could not obtain the primary text (paywalled, and no open-access mirror found); based on abstracts, citing surveys, or search-engine-synthesized summaries of the paper. Treated as lower-confidence and flagged explicitly at each use.

This distinction matters concretely: several **[secondary]** summaries encountered while researching this document described this project's own VRX wave model as "JONSWAP." Reading the actual installed source code (`ros2_ws/src/vrx/vrx_gz/src/Wavefield.cc`) shows this is wrong — it is the older Pierson-Moskowitz spectrum. That correction is detailed below and is exactly the kind of error this document exists to avoid repeating.

---

## 0. This project's own wave model (ground truth, verified from source)

Before comparing against the literature, here is precisely what this repository's simulation does today, verified by reading the installed code rather than assumed from documentation.

**Installed version:** VRX `jazzy` branch, tag `v3.1.2` (1 commit ahead), confirmed via `git describe --tags` in `ros2_ws/src/vrx`.

**Wave-height model — a port of `asv_wave_sim` (Rhys Mainwaring).** `vrx_gz/src/Wavefield.cc`/`.hh` implements a **sum of `N` trochoidal (Gerstner) component waves**, each parametrized by amplitude `a_i`, wavenumber `k_i`, angular frequency `ω_i`, phase `φ_i`, direction `d_i`, and a shared steepness factor `q ∈ [0,1]` (0 = pure sinusoid, 1 = full Gerstner trochoid — sharper crests, flatter troughs). Two ways to generate the `N` components are implemented, selected by the SDF `<model>` field:

- **`CWR`** (constant wavelength-to-amplitude ratio): components are geometric-scale copies of one mean wave (`scaleFactor = scale^n` for `n` centered on zero), directly analogous to how flight-sim/graphics ocean models build a multi-octave wave field without a physical spectrum at all (`Wavefield.cc:169-213`).
- **`PMS`**: component amplitudes are sampled from the **Pierson-Moskowitz spectrum** (`Wavefield.cc:216-222`):

  ```
  P(ω) = α g² / ω⁵ · exp(−5/4 · (ω_p / ω)⁴),   α = 0.0081
  ```

This is the *classic one-parameter* P-M spectrum (Pierson & Moskowitz, 1964) — it takes only the spectral peak frequency `ω_p` (derived from the SDF `<period>` field) as input; there is no independent significant-wave-height parameter and no peak-enhancement factor. Three components are sampled at `ω_p·(1−1/scale)`, `ω_p`, and `ω_p·scale` (`Wavefield.cc:242-246`), each weighted by an ad hoc `<gain>` multiplier (`a = gain · sqrt(2·P(ω)·Δω)`) that is **not part of the P-M spectrum itself** — it's a VRX-added knob to decouple visual amplitude from the physically-implied one. **This is why the SDF field is literally named `PMS`, not `JONSWAP`** (`Wavefield.hh:113-115`): JONSWAP (Hasselmann et al., 1973) is a *different, later* spectrum that adds a peak-enhancement factor `γ` and allows significant wave height and peak period to be set semi-independently (for fetch-/duration-limited seas, not just a "fully developed" sea). Several web sources describing VRX call the model "JONSWAP" — that is incorrect for the version of VRX installed here; it is worth flagging in case that distinction matters for a paper's related-work section.

Wavenumber uses the **deep-water dispersion relation** `k = ω²/g` (`Wavefield.cc:307-312`) — appropriate for the harbor scenario (`sydney_regatta`) but an assumption worth remembering if a shallow-water scenario is ever added.

Waves ramp up smoothly from a flat start via an exponential factor `(1 − exp(−t/τ))` (`Wavefield.cc:769`, `τ` default 2.0 s), avoiding an unphysical instantaneous "waves snap on" transient.

**Height query has two implementations**, both in `Wavefield.cc`: `ComputeDepthSimply` (closed-form, valid only when `steepness = 0`, i.e. sinusoidal — used at runtime for buoyancy sampling) and `ComputeDepthDirectly` (a 2D multivariate Newton solver, `Wavefield.cc:773-883`, needed because a true Gerstner wave's horizontal particle displacement means "the point on the surface above `(x,y)`" is not simply `height(x,y)` — the surface point that *displays* height at horizontal position `(x,y)` was itself displaced there from a different rest position, so recovering it requires inverting that displacement).

**Physics coupling — two different plugins, verified from source:**
- `vrx::Surface` (`vrx_gz/src/Surface.cc`/`.hh`) — a simplified cylindrical-hull approximation, sampling wave height at a small number of named points along the hull and computing buoyancy via circular chord-segment area (`CircleSegment()`, `Surface.hh:157-158`). This is what the WAM-V vessel itself uses: `vrx_urdf/wamv_gazebo/urdf/dynamics/wamv_gazebo_dynamics_plugin.xacro` attaches two `Surface` instances (one per hull, 2 sample points each, `hull_length=4.9 m`, `hull_radius=0.213 m`).
- `vrx::PolyhedraBuoyancyDrag` (`PolyhedraBuoyancyDrag.cc`/`.hh`) — exact polyhedral buoyancy per **Erin Catto, "Exact Buoyancy for Polyhedra," *Game Programming Gems 6*, 2006** (cited directly in `PolyhedraBuoyancyDrag.hh:32-33`), used for the smaller marker buoys in each world.

Both plugins subscribe to a shared `Wavefield` instance in "secondary mode," receiving parameter updates over the Gazebo Transport topic `/vrx/wavefield/parameters` — meaning every buoyant object in a world reads from one common wave-height field rather than each computing its own, so the vessel and any nearby buoy see a mutually consistent sea surface.

**Vessel hydrodynamic damping is a separate plugin, not part of the wave model.** `vrx::SimpleHydrodynamics` (wired alongside `Surface` in the same xacro) supplies added-mass and linear/quadratic drag coefficients (`xDotU`, `xU`, `xUU`, etc. — a standard Fossen-style maneuvering-model parametrization). Waves push the hull up and down via buoyancy; `SimpleHydrodynamics` is what damps the resulting motion.

**What this project's own launched world (`sydney_regatta.sdf`) actually sets, verified by reading the world file directly** (`vrx_gz/worlds/sydney_regatta.sdf:603-636`): a `vrx::PublisherPlugin` re-broadcasts a fixed `gz.msgs.Param` message every 2.0 s on `/vrx/wavefield/parameters` with only four fields set explicitly — `direction=0.0`, `gain=0.3`, `period=5 s`, `steepness=0`. Every other field (`model`, `number=3`, `scale=1.1`, `angle=0.4`, `tau=2.0`, `amplitude=0.0`) is left at the `WavefieldPrivate` constructor's default (`Wavefield.cc:50-67`), which is `model="PMS"`. Net effect: **this project's currently-running simulation has an active 3-component Pierson-Moskowitz wave field with a 5 s spectral peak period, `steepness = 0` (i.e., linear/sinusoidal component waves — the full nonlinear Gerstner trochoid shape is available but not currently turned on), and a fairly low `gain` (0.3), all traveling in the local +x direction.** This independently confirms the ~0.16 m vessel z-bobbing measured earlier (`scripts/analyze_manipulator_coupling.py`, logged in `docs/implementation-status.md`) is genuine wave-induced motion rather than settling noise, since an active, correctly-parametrized wave field is in fact running.

**Implication for M4 (disturbance config).** Two knobs are currently unused and physically meaningful for a disturbance-severity sweep without touching plugin code: `steepness` (0→1, sinusoidal→full Gerstner nonlinearity) and `gain`/`period` (amplitude/frequency content). A newer upstream VRX/`asv_wave_sim` branch also offers an **FFT-based** wave generator (see §6) — not present in the installed `v3.1.2` checkout (confirmed: `grep -r "FFT\|fftw"` over the whole `vrx` source tree returns nothing) — worth knowing about but not a near-term blocker, since the current PMS/Gerstner model already exposes real spectral content.

---

## 1. Bingham, B., Agüero, C., McCarrin, M., Klamo, J., Malia, J., Allen,
K., Lum, T., Rawson, M., and Waqar, R. (2019). "Toward Maritime Robotic Simulation in Gazebo." *Proceedings of MTS/IEEE OCEANS*, Seattle, WA.

**Access:** **[secondary]**. Full text is paywalled on IEEE Xplore (returned HTTP 402/418 to automated fetch); ResearchGate returned HTTP 403; an NPS-hosted copy (Bingham is NPS-affiliated) redirected to an institutional login. Description below is synthesized from multiple search-engine summaries of the paper plus this project's own reading of the codebase this paper describes (§0), which is the stronger source for implementation detail regardless.

**What it is:** the founding VRX paper — the general-purpose, Gazebo-based USV simulation later adopted as the standard testbed for the Maritime RobotX competition, and the simulator this project builds on. Per the abstract-level description available, it identifies a real gap (Gazebo had strong terrestrial/aerial support but little for marine vehicles) and addresses it with a wave model built from a **summation of regular component waves**, with **Gerstner-wave** kinematics (chosen for "sharper crests, broader troughs" over pure sinusoids) and component-wave amplitudes sampled from a **Pierson-Moskowitz spectrum**.

**Correction to secondary sources:** several search summaries describe this as a "JONSWAP" spectrum. Section 0 above shows the actual shipped code (which is presumably what the paper itself documents, since VRX's own SDF field is literally named `PMS`) implements the plain Pierson-Moskowitz spectrum, not JONSWAP. Since the paper's full text wasn't accessible, this can't be checked against the paper's own wording directly — but the implementation it describes is unambiguous, so this review defers to the code.

**Relevance:** direct — this is this project's own simulator's origin paper.

---

## 2. Manhães, M. M. M., Scherer, S. A., Voss, M., Douat, L. R., and
Rauschenbach, T. (2016). "UUV Simulator: A Gazebo-based Package for Underwater Intervention and Multi-Robot Simulation." *OCEANS 2016 MTS/IEEE Monterey*. DOI: 10.1109/OCEANS.2016.7761080.

**Access:** **[secondary]** for the paper text itself; **[source-verified]** for the disturbance-modeling description, read directly from the project's own documentation (`uuvsimulator.github.io/packages/uuv_simulator/docs/tutorials/disturbances/`).

**What it is:** a Gazebo package for **underwater** (not surface) vehicle and manipulator simulation, implementing **Fossen's equations of motion** for marine craft (added-mass, linear/quadratic damping, buoyancy from volume/fluid-density/center-of-buoyancy — the standard 6-DOF underwater maneuvering model, distinct from VRX's per-point wave-height buoyancy sampling).

**Wave modeling: there isn't any.** This is the most useful negative data point in this review. UUV Simulator's environmental disturbance is **current**, not surface waves: current speed, horizontal angle, and vertical angle are each modeled as an independent **Gauss-Markov process** (per Fossen, 2011) — a first-order stochastic process that wanders around a mean with bounded noise, not a spectral wave model at all. This makes sense for the package's target use case (subsurface vehicles/manipulators, where surface wave orbital motion is negligible at depth) but confirms that "the Gazebo maritime robotics package" answer to "how are waves modeled" is genuinely different depending on whether the target vehicle operates at or below the surface — this project's WAM-V+arm setup is a surface case, so VRX's spectral wave-height model (§0), not UUV Simulator's Gauss-Markov current, is the correct reference class.

**Relevance:** contrast case — clarifies that "marine Gazebo simulator" is not a single wave-modeling answer; the surface/subsurface distinction determines whether a spectral wave model is even present.

---

## 3. Cieślak, P. (2019). "Stonefish: An Advanced Open-Source Simulation
Tool Designed for Marine Robotics, With a ROS Interface." *MTS/IEEE OCEANS 2019*, Marseille. DOI: 10.1109/OCEANSE.2019.8867434.

**Access:** **[secondary]** for the paper text; **[source-verified]** for the wave-modeling description, read directly from the Stonefish source code (`github.com/patrykcieslak/stonefish`, `master` branch, files listed below).

**What it is:** a C++ marine-robotics simulator built on **Bullet Physics**, distinguished by "advanced hydrodynamics computation based on actual geometry" and detailed ocean/underwater rendering.

**Wave modeling: GPU FFT spectral synthesis, and — importantly, verified by reading the force-computation code, not just the shaders — the same surface used for rendering is also queried for physics.** The shader pipeline (`Library/shaders/oceanInit.frag`, `oceanFFTX/Y.frag`, `oceanSpectrum.frag`, `oceanVariance.frag`) computes an FFT-based ocean height field, with the code's own header comment attributing the technique to **Dupuy, J. and Bruneton, E., "Real-time Animation and Rendering of Ocean Whitecaps"** — itself a GPU-optimized descendant of the Tessendorf FFT-ocean lineage widely used in computer graphics. The only physical input parameter is `wind` — wind speed in m/s "at 10 m above surface" (`OpenGLOcean.h:45`), the same reference-height convention used by Pierson-Moskowitz and JONSWAP, consistent with an underlying wind-driven (Phillips-type) spectrum, though the string "Phillips" itself does not appear literally in the fetched shader source.

Critically, `Ocean::GetDepth()`-adjacent code (`Library/src/entities/forcefields/Ocean.cpp:151-158`) calls `glOcean->ComputeWaveHeight(point.x, point.y)` and computes submersion depth as `point.z − waveHeight` when waves are enabled — i.e., the **physics buoyancy calculation samples the same FFT wave surface used for visual rendering**, rather than routing disturbance through a separate abstraction. (Fluid *current*, by contrast, is a separate `VelocityField` system, `Ocean.cpp:183-198` — current and wave-elevation are represented independently, similar in spirit to VRX's `SimpleHydrodynamics`/wave-height split in §0.)

**Contrast with this project's simulator:** both VRX and Stonefish couple a queryable wave-height function directly into per-point buoyancy, which is the architecturally important similarity. They differ entirely in *how* that height function is generated: VRX/`asv_wave_sim` uses a small number (default 3) of closed-form spectral component waves summed analytically (cheap, CPU-side, exact); Stonefish uses a GPU FFT over a much larger grid (visually richer, wind-parametrized, but a fundamentally different — and harder to introspect/reproduce bit-for-bit across runs — generation method).

**Relevance:** useful alternative-implementation reference, not currently used by this project.

---

## 4. Nader, A., Ud Din, M., Irfan, M., and Hussain, I. (2024). "Object
Manipulation in Marine Environments using Reinforcement Learning." *15th IFAC Conference on Control Applications in Marine Systems, Robotics and Vehicles (CAMS 2024)*. arXiv:2406.03223.

**Access:** **[secondary]** — automated PDF text extraction failed (binary/compressed stream, not cleanly parseable by the fetch tool used); description below is from a search-engine synthesis of the paper, so treat wave-model specifics as lower-confidence than the other entries here. Flagged for a follow-up manual read if this paper becomes load-bearing for a related-work section.

**What it is — the most directly relevant paper found: a USV with a manipulator arm, doing object retrieval under wave disturbance, trained with RL.** Two simulators are used: **PyBullet** for RL training and the **MBZIRC maritime simulator** for more realistic testing/validation — a train-cheap/validate-expensive split.

**Wave modeling:** sea conditions are specified via the **World Meteorological Organization (WMO) sea state code** — a discrete 0–9 categorical scale (calm, slight, moderate, rough, ...) rather than a continuous spectral parametrization like Pierson-Moskowitz or JONSWAP. The reported experiment uses **sea state 2**. This is a coarser, more categorical treatment of "how rough is the water" than this project's own continuous `period`/`gain`/`steepness` knobs (§0) — useful context for M4's disturbance config: WMO sea-state buckets could be a convenient way to *label* disturbance-severity conditions for readability/reporting even while the underlying continuous VRX parameters are what's actually varied.

**Task/RL:** dock-based object retrieval; **Soft Actor-Critic (SAC)**, described as an actor-critic method with the actor learning a policy that "minimizes an objective function" while the critic evaluates it. Reported result: **80% success rate under sea state 2**, trained on a static-target task and found to generalize to wave-induced disturbance without needing wave-specific training.

**Relevance:** closest analogue to this project's own task (arm on a disturbed marine platform, learned grasp/retrieval policy) found in this search. Worth a manual full-text read before citing quantitatively, given the access caveat above.

---

## 5. Herland, S. and Bach, K. (2023). "Vessel-to-Vessel Motion
Compensation with Reinforcement Learning." *Proceedings of the AAAI Conference on Artificial Intelligence* (IAAI-23 track). Copyright 2023, AAAI.

**Access: [primary-text-verified].** Full PDF located and read in full (`folk.idi.ntnu.no/kerstinb/paper/2023_IAAI_HerlandBach.pdf`) — the most rigorously verified entry in this review.

**What it is:** a robotic arm mounted on one floating vessel must hold its end effector at a fixed pose *relative to a second, independently moving vessel* — i.e., cancel out the *relative* motion between two wave-disturbed platforms, rather than reject motion relative to a fixed world frame. Directly relevant framing for this project's own "arm on a disturbed vessel" setup, even though the target here is a second vessel rather than a free-floating graspable object.

**Wave/motion modeling — the methodologically distinct approach worth highlighting: waves are not simulated live in their own physics engine at all.** Vessel motion trajectories are **pre-generated by an external, separate high-fidelity marine-operations simulator, SIMO** (SINTEF Ocean), modeling a real 25.5 m aquaculture service vessel, reused from an earlier work (Brandt et al., 2022) that used real ship motion data provided by SINTEF Ocean. Sea conditions are parametrized with the **JONSWAP spectrum** (Hasselmann et al., 1973) via **exactly two independent parameters, significant wave height `H_s` and peak period `T_p`**, at four increasing-severity sea states:

| Sea state | `H_s` [m] | `T_p` [s] |
|---|---|---|
| `W_a` | 1.0 | 6.0 |
| `W_b` | 1.5 | 8.0 |
| `W_c` | 2.0 | 9.0 |
| `W_d` | 2.5 | 10.0 |

(their Table 1 — genuinely JONSWAP, both parameters independently set, unlike VRX's one-parameter P-M default in §0). The resulting **time series of vessel position/orientation is then replayed** as a boundary condition into a MuJoCo physics simulation (via the DeepMind Control Suite) of the two vessels and a UR10e arm — the arm and its controller experience realistic wave-frequency motion without MuJoCo (or anything else in their pipeline) ever computing a wave field or buoyancy force. This is a **precompute-then-replay** architecture, the polar opposite of VRX's **live spectral field sampled every physics step** architecture in §0. Two consequences they note explicitly: (1) since each episode slices two independent random windows from one vessel's motion log to stand in for two separate vessels, the two vessels' motions are *not* phase-correlated the way real co-located vessels in the same sea state would be — acknowledged as making the prediction problem *harder* than reality, not easier, so treated as an acceptable conservative simplification rather than hidden; (2) they scale the real vessel trajectory down by `λ=10` (position ÷ λ, sample rate × √λ, preserving linear accelerations) to fit a lab-scale arm's workspace, explicitly flagging this as making the control problem "slightly harder" (≈44% relative motion speedup) rather than easier.

**RL/task:** hybrid controller — a conventional Inverse Kinematics (IK) + PD controller does the baseline tracking, and a **PPO**-trained (recurrent, LSTM-based) RL policy predicts a *small corrective transform* on top of the IK target (residual/corrective policy learning, not learning the whole control problem from scratch). Reward is the negated sum of positional and orientational tracking error. Across all four sea states and five tested actuation-delay configurations, the RL correction reduced average positional error by roughly 43–90% versus the IK-only baseline, with the largest gains under the longest actuation delays.

**Explicitly stated limitations (their own Discussion section, not inferred):** results are simulation-only, no sim-to-real gap has been characterized; a small-scale (λ=10) arm was used, and full-scale-arm vibration effects are flagged as an open question for future work; the two-vessel decorrelation assumption above.

**Relevance:** the strongest methodological point of comparison for this project's own disturbance-generation choice. It demonstrates a viable alternative to "simulate the wave physics live": generate (or reuse) a realistic vessel-motion time series once, offline, from a higher-fidelity source, then replay it as a kinematic boundary condition. Worth keeping in mind for M4/M10 (robustness sweep) — if VRX's live spectral model ever proves too limited (e.g., can't reach a desired motion realism, or reproducibility/determinism across seeds becomes an issue), a recorded-trajectory-replay fallback is a proven, published approach, not a novel risk.

---

## 6. Related but lower-confidence / not fully read

**Vasiljević, G. et al. "Robust Ship-to-Ship Object Pick-up with a 6-DoF Robotic Arm Based on Force/Torque Measurement and Gripper Design."** *Journal of Field Robotics* (Wiley). **[secondary, unverified]** — full text returned HTTP 402/403 to every access attempt (Wiley paywall, and an Authorea-hosted preprint copy also refused access). Search-summary-level description: ship-to-ship object transfer between two docked vessels using a 6-DoF arm, force/torque-based sensing and gripper design to compensate for relative-motion disturbance, tested in both lab and real-world maritime conditions. **Not citing any wave-model specifics from this one** — the access failure means nothing here should be treated as verified. Listed only so a future pass knows this paper exists and is worth another access attempt (e.g. via an institutional subscription) rather than being re-discovered from scratch.

**`asv_wave_sim` main branch (Mainwaring, ongoing) — FFT wave mode.** **[source-verified, but not the version installed here]**. Search results and the project's own README (not the installed `v3.1.2` VRX fork's vendored copy, but the upstream `srmainwaring/asv_wave_sim` repository directly) confirm a `main`-branch rewrite supports three wave-generation algorithms selectable by name — `sinusoid`, `trochoid`, and `fft` — using CGAL for mesh handling and FFTW for the transform, and drops the ROS dependency entirely (targets Gazebo Garden). This is the likely upstream source of the FFT capability VRX itself doesn't yet expose in the installed `v3.1.2` checkout (§0). Not independently verified against that repository's own source for this review — flagged for later if the FFT mode becomes relevant (e.g. wanting broader-spectrum, more visually complex seas than the 3-component PMS/CWR sum currently gives).

---

## 7. Primary wave-spectrum references (the math underneath all of the above)

Not simulator papers, but the original sources every spectral model above ultimately cites. Included for completeness/citability, not read in full here — standard, load-bearing oceanography references:

- **Pierson, W. J. and Moskowitz, L. (1964).** "A proposed spectral form for fully developed wind seas based on the similarity theory of S. A. Kitaigorodskii." *Journal of Geophysical Research*, 69(24). — the origin of the one-parameter P-M spectrum VRX's `PMS` mode implements (§0).
- **Hasselmann, K. et al. (1973).** "Measurements of wind-wave growth and swell decay during the Joint North Sea Wave Project (JONSWAP)." *Deutsches Hydrographisches Institut*. — the origin of the JONSWAP spectrum used by Herland & Bach (§5), *not* the spectrum this project's currently-installed VRX uses (a correction worth remembering, per the §0/§1 discrepancy noted above).
- **Tessendorf, J. (2001, revised in later course notes).** "Simulating Ocean Water." *SIGGRAPH course notes*. — the foundational FFT ocean-surface method whose lineage both Stonefish's technique (§3, via Dupuy & Bruneton) and `asv_wave_sim`'s FFT mode (§6) descend from.
- **Catto, E. (2006).** "Exact Buoyancy for Polyhedra." *Game Programming Gems 6*, Charles River Media, pp. 175–187. — the buoyancy-integration method VRX's `PolyhedraBuoyancyDrag` plugin implements (§0).

---

## 8. Summary comparison table

| Source | Wave generation | Spectrum | Physics coupling | Live or precomputed |
|---|---|---|---|---|
| **This project (VRX v3.1.2)** | 3-component Gerstner sum | Pierson-Moskowitz (`PMS`) or geometric (`CWR`) | Per-point height sampled directly into buoyancy (`Surface`/`PolyhedraBuoyancyDrag`) | Live, closed-form |
| UUV Simulator (Manhães 2016) | *none* — Gauss-Markov current only | N/A | Fossen 6-DOF current force, no surface height field | Live, stochastic process |
| Stonefish (Cieślak 2019) | GPU FFT (Dupuy & Bruneton method) | Wind-parametrized (Phillips-type, implied) | Same FFT height field sampled for both rendering and buoyancy | Live, spectral/FFT |
| Nader et al. 2024 | Not specified beyond WMO sea-state code | WMO categorical scale (0–9) | Not detailed (secondary source only) | Presumed live in MBZIRC sim |
| Herland & Bach 2023 | External SIMO simulation, replayed | JONSWAP (independent `H_s`, `T_p`) | Kinematic boundary condition, no in-loop wave physics | **Precomputed, replayed** |
| `asv_wave_sim` main (upstream) | Selectable: sinusoid / trochoid / FFT | Depends on mode | Same family as VRX, extended | Live |

---

## Implications for this project

1. **The "JONSWAP vs. Pierson-Moskowitz" distinction is not academic for this project** — it determines whether M4's disturbance config can set significant wave height and peak period independently (JONSWAP-style, what Herland & Bach's sea states table does) or only a single coupled peak-period knob with an ad hoc gain fudge factor (what this project's installed VRX actually does today, per §0). If independent `H_s`/`T_p` control matters for the eventual robustness sweep (M10), that's either a case for switching VRX's `<model>` to something JONSWAP-like (not available in the installed version — would require patching `Wavefield.cc`'s `Pm()` to add the JONSWAP peak-enhancement factor `γ`) or accepting the one-parameter P-M coupling as a known, documented limitation.
2. **Unused, already-available knobs**: `steepness` (currently 0, i.e. linear waves — full Gerstner nonlinearity is one SDF field away) and `gain` are free severity-sweep parameters with zero new code required.
3. **Herland & Bach's precompute-and-replay pattern** is a credible fallback if VRX's live spectral model ever becomes a bottleneck for reproducibility (same seed → bit-identical wave field across runs is not yet verified for this project's setup) or for wanting real-recorded-motion realism rather than a synthetic spectrum.
4. **WMO sea-state labels** (Nader et al.) are a reasonable human-readable severity label to report *alongside* the actual VRX parameters used, without adopting them as the underlying control variable.
