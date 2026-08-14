# Bayesian Grid Search, Mutual-Information Hunt, and Inverse-Cube Dipole Localization

**Internal algorithm / study paper** matching `version_3`  
(`bayes_core.py`, `info_gain.py`, `dipole_fit.py`)

**PDF:** upload [`docs/algorithm_paper_overleaf.zip`](../algorithm_paper_overleaf.zip) to Overleaf → `main.tex` → pdfLaTeX.

---

## Abstract

This paper explains **exactly** the three estimation/planning algorithms in the cooperative ASV magnetic survey:

1. **Bayesian grid search** — discrete occupancy PMF with a soft \(1/r^3\) detection likelihood  
2. **Mutual-information hunt** — next pose maximizes expected entropy reduction  
3. **Inverse-cube dipole LS** — Levenberg–Marquardt fit of \(A/(r^3+s^3)\) for \((t_x,t_y,A)\)

We state the physical inverse-cube law, the probabilistic model, the algorithms, why they are consistent, the high-level parameters, and how a **peer ASV** uses the same dipole/belief products to verify.

---

## 1. Engineering picture

The target is unknown. Each ASV measures cleaned anomaly \(a\) (nT). Estimators never get planted truth \(\mathbf{t}^\star\).

| Layer | Role | Output |
|-------|------|--------|
| Bayes grid | Coarse probabilistic “where is \(T\)?” | peak, \(p^\star\), centroid, \(\sigma\) |
| MI hunt | Where to go next | waypoint maximizing \(I(T;Z)\) |
| Dipole inverse | Fine physics fit | `/swarm/belief/fix` (metres) |

Multi-ASV: all anomalies → **one** shared `bayes_fusion` map. Discoverer hunts/spirals; **peer** verifier orbits the declared candidate.

```
SEARCH (strips) → Bayes HITs concentrate b
 → MI hunt → spiral densify samples
 → declare candidate (centroid)
 → peer VERIFY (same dipole/Bayes evidence)
 → COMPLETE; score ‖t_fix − t*‖
```

---

## 2. Physical model: why \(1/r^3\) (inverse cube, not inverse square)

Magnetostatic dipole far field:

\[
\mathbf{B}(\mathbf{r})\propto \frac{3(\mathbf{m}\cdot\hat{\mathbf{r}})\hat{\mathbf{r}}-\mathbf{m}}{r^3}
\quad\Rightarrow\quad |\mathbf{B}|\sim \frac{1}{r^3}.
\]

This is **not** Coulomb \(1/r^2\) (monopole). For a mainly vertical dipole seen in \(B_z\), the scalar anomaly follows a radial \(1/r^3\) envelope.

**Regularized plant** (`dipole.py` / `dipole_fit.py`):

\[
a(\mathbf{p};\,\mathbf{t},A,s)=\frac{A}{r^3+s^3},\qquad r=\|\mathbf{p}-\mathbf{t}\|_2.
\]

YAML: \(A=4\times10^5\,\mathrm{nT\cdot m^3}\), \(s=20\,\mathrm{m}\) ⇒ overhead peak \(A/s^3=50\,\mathrm{nT}\). At \(r=s=20\,\mathrm{m}\), \(a=25\,\mathrm{nT}\). The \(s^3\) term keeps the field finite at \(r=0\) (sensor height / source size). Noise ~ \(3\,\mathrm{nT}\); HIT at \(15\,\mathrm{nT}\) ≈ \(5\sigma\).

---

## 3. Bayesian grid search

### State

Lake \(300\times300\,\mathrm{m}\), cells \(\Delta=10\,\mathrm{m}\), origin \((-150,-150)\), \(N=900\) cells. Unknown target cell \(T\in\{1,\ldots,N\}\):

\[
b_k=\Pr(T=k),\quad \sum_k b_k=1.
\]

**Prior:** uniform \(b_k^{(0)}=1/N\).

### Observation

\[
z=\begin{cases}
\mathrm{HIT} & a_{\mathrm{cl}}\ge 15\,\mathrm{nT}\\
\mathrm{MISS} & a_{\mathrm{cl}}\le 5\,\mathrm{nT}\\
\mathrm{ABSTAIN} & \text{else or uncalibrated}
\end{cases}
\]

`hit_only: true` ⇒ MISS is ignored (no update). Conservative: weak “no detect” does not erase mass.

### Likelihood (matched to \(1/r^3\))

ASV at \(\mathbf{x}\), cell centre \(\mathbf{c}_k\), \(d_k=\|\mathbf{x}-\mathbf{c}_k\|\):

\[
P_{\mathrm{det}}(d)=P_{\mathrm{bg}}+(P_{\max}-P_{\mathrm{bg}})\frac{d_{1/2}^3}{d^3+d_{1/2}^3}
\]

\(P_{\mathrm{bg}}=0.05\), \(P_{\max}=0.95\), \(d_{1/2}=30\,\mathrm{m}\). Far: \(P_{\mathrm{det}}\to P_{\mathrm{bg}}\). Overhead: \(\to P_{\max}\). Same **shape** as the dipole envelope.

\[
\Pr(\mathrm{HIT}\mid T=k)=P_{\mathrm{det}}(d_k),\quad
\Pr(\mathrm{MISS}\mid T=k)=1-P_{\mathrm{det}}(d_k).
\]

### Bayes update (the algorithm)

HIT at \(\mathbf{x}\):

\[
b_k^+ \;\propto\; b_k^-\, P_{\mathrm{det}}(\|\mathbf{x}-\mathbf{c}_k\|),\qquad b^+\leftarrow b^+/\sum_j b_j^+.
\]

This **is** `BeliefMap.update` in `bayes_core.py`. Discrete Bayes / histogram filter (Stone-style search); not Gaussian.

**Why HITs concentrate:** a HIT near true cell \(k^\star\) has \(P_{\mathrm{det}}(d_{k^\star})\gg P_{\mathrm{det}}(d_\ell)\) for far cells, so \(b_{k^\star}\) grows after renormalization. Repeated HITs → unimodal blob.

### Summaries

- **Peak** \(\arg\max b_k\) → `/swarm/belief/peak`, \(p^\star\)  
- **Centroid** of cells with \(b_k\ge 0.5\,b_{\max}\), mass-weighted + RMS spread \(\sigma\)

Mission **prefers centroid over peak** for hunt/declare (sub-cell).

### Why it works (engineering)

1. Likelihood exponent matches physics (\(1/d^3\)) — matched, not a generic kernel  
2. Uniform prior + HIT-only + \(5\sigma\) gate → mass only near real detections  
3. All ASVs share one \(b\) (independent obs, same \(T\)) — information adds  
4. Limit: \(10\,\mathrm{m}\) cells jump; centroid + dipole LS refine metres  

### Pseudocode

```
b ← uniform(N cells)
for each MagAnomaly (x,y,a_cl) from any ASV:
  if not calibrated or 5 < a_cl < 15: continue   # ABSTAIN
  if a_cl ≥ 15:   # HIT
    for each cell k:
      b[k] *= P_det(||(x,y)-c_k||)
    b ← b / sum(b)
publish peak, p*, centroid, spread
```

---

## 4. Mutual-information hunt

### Objective

Shannon entropy \(H(b)=-\sum_k b_k\log b_k\). Next pose \(\mathbf{c}\) should **maximize expected entropy drop** = mutual information between target cell \(T\) and next binary observation \(Z\in\{\mathrm{HIT},\mathrm{MISS}\}\):

\[
I(T;Z\mid\mathbf{c})=H(b)-\mathbb{E}_Z[H(b'\mid Z,\mathbf{c})].
\]

### Predictive HIT probability

\[
p_{\mathrm{HIT}}(\mathbf{c})=\sum_k b_k\, P_{\mathrm{det}}(\|\mathbf{c}-\mathbf{c}_k\|)
\]

Then \(b^{\mathrm{HIT}}_k\propto b_k P_{\mathrm{det},k}\), \(b^{\mathrm{MISS}}_k\propto b_k(1-P_{\mathrm{det},k})\), and

\[
\mathbb{E}[H(b')]=p_{\mathrm{HIT}}H(b^{\mathrm{HIT}})+(1-p_{\mathrm{HIT}})H(b^{\mathrm{MISS}}).
\]

`mutual_information_gain` returns \(H(b)-\mathbb{E}[H(b')]\). **Always \(I\ge 0\)** (equality iff \(Z\) independent of \(T\)).

### What we actually search

Not the whole lake. Ring candidates around current pose: radii \(\{10,20,30\}\,\mathrm{m}\), 16 angles, clipped to bounds. Replan every \(5\,\mathrm{s}\). Fallback if no grid: ring point closest to peak.

### Why MI works

- Lawnmower = open-loop coverage; MI = closed-loop “go where the next reading most splits competing cells”  
- Same \(P_{\mathrm{det}}\) as Bayes → **planner and filter are model-consistent**  
- Exit to spiral: MI collapse, near peak (\(\le25\,\mathrm{m}\)), 45 steps, or \(p^\star\ge0.5\) near estimate  

### Pseudocode

```
candidates = ring(current, radii={10,20,30}, 16 angles) inside bounds
for c in candidates:
  I[c] = H(b) - (pHIT*H(b_HIT) + pMISS*H(b_MISS))
go to argmax I
```

---

## 5. Dipole inverse (Levenberg–Marquardt)

### Problem

Bayes = cell. Physics = continuum. Buffer \(\{(x_i,y_i,a_i)\}\) with \(a_i\ge10\,\mathrm{nT}\) (min 12, max 400):

\[
\min_{t_x,t_y,A}\sum_i\left(\frac{A}{r_i^3+s^3}-a_i\right)^2,\quad s=20.
\]

### Jacobians (what LM uses)

\(f_i=A/(r_i^3+s^3)\):

\[
\frac{\partial f}{\partial A}=\frac{1}{r^3+s^3},\quad
\frac{\partial f}{\partial t_x}=\frac{3A\, r\, d_x}{(r^3+s^3)^2},\quad
\frac{\partial f}{\partial t_y}=\frac{3A\, r\, d_y}{(r^3+s^3)^2}.
\]

Solve \((J^\top J+\lambda D)\delta=-J^\top\mathbf{r}\), step cap \(40\,\mathrm{m}\). Success if RMS \(<20\,\mathrm{nT}\), \(A>0\).

Warm start: centroid guess, or anomaly-weighted sample centroid; \(A\approx a(r^3+s^3)\) from strongest sample.

### Why it works

- \(1/r^3\) curvature is identifiable **if** you have **angular diversity** (spiral/orbit) — a single radial pass is poorly conditioned  
- Plant and fitter use the **same** \(A/(r^3+s^3)\) ⇒ zero-bias in sim when samples surround \(\mathbf{t}^\star\) (best \(e_{\mathrm{fix}}\approx0\,\mathrm{m}\))  
- Bayes = robust multi-ASV fusion; dipole = precise metres. Bayes **gets you there**; dipole **scores metres**  

---

## 6. How the other ASV uses dipole / belief

1. Discoverer declares **centroid** on `/swarm/verify/declare`  
2. Coordinator assigns a **different** ASV (`prefer_other_verifier: true`)  
3. Verifier orbits candidate (≤20 m, inland-capped), opposite approach  
4. Confirmation iff **all**: at-site ≤30 m, peak within 50 m of candidate, \(a_{\mathrm{cl}}\ge15\,\mathrm{nT}\), \(p^\star\ge0.30\)  
5. 4 confirms → COMPLETE. Dipole LS still updates from **all** ASV samples  

The peer does **not** use a different law. It re-samples the **same inverse-cube field** and **same Bayes map** from new geometry — that independence is why verify means something.

---

## 7. Parameter catalog (how it is modelled)

| Symbol / YAML | Value | Meaning |
|---------------|-------|---------|
| `cell_size_m` \(\Delta\) | 10 m | Bayes cell |
| map / origin | 300 m, (−150,−150) | Lake grid |
| \(P_{\mathrm{bg}},P_{\max}\) | 0.05, 0.95 | FA floor / max \(P_{\mathrm{det}}\) |
| \(d_{1/2}\) | 30 m | Mid detection range |
| \(\tau_{\mathrm{hit}},\tau_{\mathrm{miss}}\) | 15, 5 nT | HIT/MISS gates |
| `hit_only` | true | Ignore MISS updates |
| centroid frac | 0.5 | High-prob blob |
| \(A,s\) | \(4\times10^5\), 20 m | Dipole amp/soft |
| dipole min \(a\), \(n\) | 10 nT, 12 | LS buffer |
| MI radii / angles | 10/20/30 m, 16 | Hunt candidates |
| MI replan | 5 s | Hunt rate |
| enter TARGET \(p^\star\) | 0.25 × 4 ticks | Leave lawnmower |
| spiral max / inset | 80 / 10 m | Inland densify |
| verify \(n\) / orbit | 4 / 20 m | Peer confirm |

**Retune:** higher \(\tau_{\mathrm{hit}}\) → fewer false HITs. Higher \(d_{1/2}\) → longer-range, smearer blob. Smaller \(\Delta\) → finer, more CPU. Dipole \(s\) **must match plant** or LS is biased.

---

## 8. End-to-end consistency (the “proof” you tell others)

1. **Physics → likelihood:** \(B\propto 1/r^3\) ⇒ \(P_{\mathrm{det}}\) uses the same exponent  
2. **Likelihood → Bayes:** HIT update is Bayes’ rule on categorical \(T\); \(b\) stays a PMF  
3. **Bayes → MI:** MI uses the **same** \(P_{\mathrm{det}}\) to predict \(b'\); one-step entropy-optimal  
4. **Samples → dipole:** LS inverts the **same** \(A/(r^3+s^3)\) used to generate \(a\) in sim  
5. **Peer ASV:** independent pose, same field, same \(b\) and LS buffer  

**One inverse-cube world model** is used as plant, likelihood, planner, and fitter — not three unrelated hacks.

---

## 9. Study questions (quiz yourself / teammates)

1. Why \(1/r^3\) not \(1/r^2\)?  
2. What does \(s\) do at \(r=0\)?  
3. Two-cell HIT update: which cell grows?  
4. Why HIT-only? What would aggressive MISS do far from the target?  
5. Show \(I(T;Z)\ge 0\). When is \(I=0\)?  
6. Why rings 10/20/30 m instead of full-lake MI?  
7. Why spiral before dipole LS?  
8. Peak vs centroid vs dipole fix?  
9. Why can ASV1 verify what ASV3 discovered?  
10. If \(\tau_{\mathrm{hit}}=3\,\mathrm{nT}\) (noise), what fails?

---

## 10. Code map

| Algorithm | File |
|-----------|------|
| \(P_{\mathrm{det}}\), Bayes, centroid | `boat_mapping/bayes_core.py` |
| Fusion + dipole buffer | `boat_mapping/bayes_fusion.py` |
| LM dipole + Jacobians | `boat_mapping/dipole_fit.py` |
| Plant dipole | `boat_sensing/dipole.py` |
| MI planner | `boat_mission/info_gain.py` |
| FSM / verify | `mission_manager.py`, `verify_core.py` |
| Parameters | `mapping.yaml`, `mission.yaml`, `sensing.yaml` |

---

## References

1. C. E. Shannon, “A mathematical theory of communication,” *Bell Syst. Tech. J.*, 1948.  
2. K. Levenberg, “A method for the solution of certain non-linear problems in least squares,” *Q. Appl. Math.*, 1944.  
3. L. D. Stone, *Theory of Optimal Search*, 1975.  
4. T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2011.  
5. Open Robotics, ROS 2 Jazzy / Gazebo Harmonic docs.
