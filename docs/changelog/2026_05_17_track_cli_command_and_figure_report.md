# 2026_05_17 Track CLI Command and Figure Report

## 1. Purpose and Scope

This report documents the current command-line workflow for the centerline-based racing trajectory study. The emphasis is not on software implementation details, but on the numerical purpose of each command, the interpretation of the generated figures, and the mathematical structure behind the plots. The present workflow treats the given centerline as a fixed closed path, then analyzes the path geometry, the curvature discretization, and the resulting standing-start speed profile obtained from a three-pass propagation procedure.

The command entry point is `script/track_cli.py`, which exposes four user-facing commands:

1. `list-tracks`
2. `audit-track`
3. `compare-methods`
4. `analyze-track`

These commands form a sequential numerical workflow: identify a dataset, inspect the raw geometry, compare differentiation strategies for curvature, and finally propagate a speed profile and estimate lap time.

## 2. Command-Level Technical Summary

### 2.1 `list-tracks`

**Purpose.**
This command enumerates all available track datasets. Its role is administrative rather than analytical: it defines the valid problem instances on which the later numerical experiments may be executed.

**Expected outcome.**
The user receives a table of available track names. No figures are created. The output is used to select a valid track for the later geometry and speed studies.

**Technical interpretation.**
Although simple, this command anchors reproducibility. It ensures that all later results are associated with a named dataset rather than an ambiguous file selection.

### 2.2 `audit-track`

**Purpose.**
This command evaluates the raw centerline dataset before any curvature or speed-profile analysis is performed. The objective is to determine whether the sampled path is numerically well-behaved, whether it is approximately closed, whether the point spacing is sufficiently uniform, and whether the width channels are physically consistent.

**Expected outcome.**
The user receives a summary table containing total centerline length, closure gap, segment-length statistics, spacing coefficient of variation, width statistics, and the number of outlier segments. The command also produces four figures that reveal whether the raw data are already suitable for finite-difference operations or whether resampling is advisable.

### 2.3 `compare-methods`

**Purpose.**
This command compares curvature computed from the raw path against curvature computed after arc-length resampling. The objective is to show how discretization choice affects a derivative-sensitive quantity. Since curvature depends on first and second derivatives of the path, it is the first quantity that strongly exposes nonuniform sampling artifacts.

**Expected outcome.**
The user receives a numerical comparison table for the raw and resampled curvature profiles, along with an analytical circle-validation table. Three figures are generated to compare curvature traces, show the discrepancy between methods, and verify that the resampled geometry is spatially uniform along the lap.

**Technical interpretation.**
If the raw and resampled curvature profiles are nearly identical, the track is already sampled in a numerically stable way. If strong oscillations or systematic offsets appear in the raw-minus-resampled difference, then the raw discretization is contaminating the derivative estimate and resampling becomes justified.

### 2.4 `analyze-track`

**Purpose.**
This command performs the main standing-start speed analysis on a fixed path. It combines the curvature-derived lateral speed cap, the forward acceleration pass, the backward braking pass, the friction-circle feasibility check, and multiple lap-time integration formulas.

**Expected outcome.**
The user receives a speed-study table containing the total path length, friction acceleration limit, several lap-time estimates, finish speed, maximum speed, and residual checks for lateral, forward, braking, and friction-circle constraints. Four figures are created to visualize the speed field on the path, the speed profile versus arc length, the longitudinal acceleration profile, and the spread among numerical integration estimates.

**Technical interpretation.**
This is the first command that converts the geometric path into a dynamic performance estimate. Its value is twofold: it produces a baseline lap-time estimate for the centerline, and it exposes where the vehicle is curvature-limited, acceleration-limited, or braking-limited.

## 3. Figure-by-Figure Interpretation

### 3.1 Figures Produced by `audit-track`

#### (a) Centerline and Track Boundaries

**Purpose.**
To verify the geometric integrity of the track and visually confirm that the left and right boundaries are consistent with the stored widths.

**How it is plotted mathematically.**
Let the centerline sample be $\mathbf{r}_i = (x_i, y_i)$. A local tangent is approximated by the periodic centered difference

$$
\mathbf{t}_i \approx \frac{\mathbf{r}_{i+1} - \mathbf{r}_{i-1}}{\lVert \mathbf{r}_{i+1} - \mathbf{r}_{i-1} \rVert}.
$$

The corresponding unit normal is

$$
\mathbf{n}_i = (-t_{y,i},\, t_{x,i}).
$$

If $w_{L,i}$ and $w_{R,i}$ are the left and right widths, then the boundaries are approximated by

$$
\mathbf{r}_{L,i} = \mathbf{r}_i + w_{L,i}\mathbf{n}_i,
\qquad
\mathbf{r}_{R,i} = \mathbf{r}_i - w_{R,i}\mathbf{n}_i.
$$

**Expected outcome.**
The boundaries should appear smooth and remain on opposite sides of the centerline. Severe spikes, flipped normals, or self-crossing offsets would indicate either corrupted width data or poor local sampling.

**How to analyze it.**
This figure is primarily qualitative. It is used to detect obvious geometric pathologies before derivative-based analysis begins.

#### (b) Segment Lengths vs Segment Index

**Purpose.**
To assess sampling uniformity along the track.

**How it is plotted mathematically.**
For each closed-loop segment,

$$
\Delta s_i = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2},
$$

with the final segment closing the loop from the last point back to the first.

The plot displays $\Delta s_i$ as a function of segment index $i$.

**Expected outcome.**
For a well-sampled path, the curve should fluctuate mildly around a dominant mean value. Large spikes suggest local undersampling, repeated points, or inconsistent data density.

**How to analyze it.**
This figure supports a decision about whether raw finite differences are trustworthy. High variability in $\Delta s_i$ implies that arc-length resampling is numerically preferable.

#### (c) Segment Length Histogram

**Purpose.**
To summarize the distribution of segment lengths.

**How it is plotted mathematically.**
The same $\Delta s_i$ values are grouped into bins, producing an empirical distribution of local spacing.

**Expected outcome.**
The histogram should be narrow and approximately unimodal if the path is sampled consistently. A wide or multi-peaked histogram suggests mixed spacing scales in the raw dataset.

**How to analyze it.**
This figure complements the segment-length trace by distinguishing isolated outliers from a systematic spacing problem.

#### (d) Track Widths vs Arc Length

**Purpose.**
To inspect whether the left and right widths vary smoothly around the lap and remain physically plausible.

**How it is plotted mathematically.**
The cumulative arc-length coordinate is

$$
s_0 = 0,
\qquad
s_k = \sum_{i=0}^{k-1} \Delta s_i.
$$

The right and left width channels are then plotted against $s$.

**Expected outcome.**
The width profiles should remain positive and vary smoothly. Abrupt jumps may indicate either a real geometric feature or a data issue that should be checked before later optimization work.

**How to analyze it.**
This figure provides context for later trajectory optimization, because spatial width variation defines the admissible motion corridor around the centerline.

### 3.2 Figures Produced by `compare-methods`

#### (a) Curvature Method Comparison

**Purpose.**
To compare curvature obtained from raw sampling and curvature obtained after arc-length resampling.

**How it is plotted mathematically.**
For a planar curve parameterized by arc length $s$, curvature is

$$
\kappa(s) = \frac{x'(s)y''(s) - y'(s)x''(s)}{\left(x'(s)^2 + y'(s)^2\right)^{3/2}}.
$$

The implementation approximates $x'(s)$, $y'(s)$, $x''(s)$, and $y''(s)$ using periodic centered finite differences on a closed loop with uniform spacing $\Delta s$:

$$
x'_i \approx \frac{x_{i+1} - x_{i-1}}{2\Delta s},
\qquad
x''_i \approx \frac{x_{i+1} - 2x_i + x_{i-1}}{\Delta s^2},
$$

with analogous formulas for $y$.

The figure plots $\kappa$ against normalized lap position $s/L$ for both discretizations.

**Expected outcome.**
If raw spacing is already nearly uniform, the two curves should overlap closely. If not, the raw curve may show artificial oscillations or local bias.

**How to analyze it.**
Agreement between the two traces supports using the raw data directly. Disagreement justifies the resampled geometry for any derivative-sensitive analysis.

#### (b) Raw Minus Resampled Curvature

**Purpose.**
To isolate the numerical discrepancy between the two curvature estimates.

**How it is plotted mathematically.**
At matched sample indices,

$$
\Delta \kappa_i = \kappa_i^{\text{raw}} - \kappa_i^{\text{resampled}}.
$$

This difference is plotted against normalized sample index.

**Expected outcome.**
The curve should remain close to zero if the sampling choice has limited influence. Large excursions identify regions where raw-spacing irregularity materially changes the local curvature estimate.

**How to analyze it.**
This figure is valuable because it transforms a visual overlap judgment into a discrepancy signal. It shows where the numerical method, rather than the geometry itself, is responsible for the curvature variation.

#### (c) Arc-Length Resampling Check

**Purpose.**
To verify that resampled points are distributed evenly along the closed path.

**How it is plotted mathematically.**
The raw path is first represented by cumulative arc length. New sample locations are then generated at nearly constant spacing,

$$
s_j^{\ast} = j\,\frac{L}{N}, \qquad j=0,1,\dots,N-1,
$$

and the coordinates are interpolated onto those positions. The plot overlays the original centerline with the resampled points.

**Expected outcome.**
The resampled points should appear visually uniform around the lap, including through corners.

**How to analyze it.**
This figure checks that the resampling procedure improved numerical regularity without visibly distorting the path geometry.

### 3.3 Figures Produced by `analyze-track`

#### (a) Reference Trajectory Colored by Speed

**Purpose.**
To provide a spatial view of the final speed solution around the track.

**How it is plotted mathematically.**
Each path segment is assigned the solved speed from the final three-pass result, and a colormap is applied to the line segments. In addition, event markers are placed where the longitudinal acceleration changes state from neutral to accelerating or from neutral to braking.

The longitudinal acceleration on segment $i$ is approximated by

$$
a_{x,i} = \frac{v_{i+1}^2 - v_i^2}{2\Delta s_i}.
$$

**Expected outcome.**
High-speed straights should appear in warmer colors, while tight corners should appear in cooler colors. Red event markers should appear where throttle application begins, and blue markers should appear where braking begins.

**How to analyze it.**
This is the most intuitive dynamic overview. It reveals where the circuit is speed-limited by curvature, where acceleration is sustained, and where braking transitions occur spatially.

#### (b) Standing-Start Speed Profile

**Purpose.**
To compare the local lateral speed cap, the forward pass, and the final speed solution along arc length.

**How it is plotted mathematically.**
The lateral acceleration requirement is

$$
a_y = v^2 |\kappa|.
$$

Hence the curvature-based speed cap is

$$
v_{\text{lat},i} = \sqrt{\frac{a_{y,\text{lim}}}{|\kappa_i| + \varepsilon}},
$$

where

$$
a_{y,\text{lim}} = \min(a_{y,\max}, a_{\text{fric}}).
$$

The forward pass applies the discrete kinematic relation

$$
v_{i+1}^2 = v_i^2 + 2a_{x,i}\Delta s_i,
$$

subject to the engine cap, lateral speed cap, and friction-circle feasibility. The backward pass imposes the analogous braking constraint from the end of the lap toward the start.

**Expected outcome.**
The forward-pass curve should begin at zero and rise until it is limited by either lateral capability or braking feasibility. The final speed profile should lie below both the forward-pass and lateral-cap curves wherever necessary.

**How to analyze it.**
The plot reveals which sections are curvature-limited and which sections are acceleration-limited. It also shows whether braking constraints propagate upstream from severe corners.

#### (c) Longitudinal Acceleration vs Arc Length

**Purpose.**
To show where the vehicle is accelerating, coasting, or braking.

**How it is plotted mathematically.**
For each segment midpoint,

$$
a_{x,i} = \frac{v_{i+1}^2 - v_i^2}{2\Delta s_i}.
$$

Positive values indicate acceleration, negative values indicate braking, and values near zero indicate locally speed-capped or coasting behavior.

**Expected outcome.**
Long positive regions should correspond to straights or corner exits, while negative regions should appear before demanding corners.

**How to analyze it.**
This figure converts the speed profile into control phases. It is particularly useful for identifying where the vehicle leaves the acceleration-limited regime and enters the braking-limited regime.

#### (d) Lap-Time Integration Comparison

**Purpose.**
To compare several numerical quadrature formulas applied to the same speed solution.

**How it is plotted mathematically.**
Lap time is computed from

$$
T = \int_0^L \frac{1}{v(s)}\,ds.
$$

The command compares several discrete approximations:

$$
T_{\text{left}} = \sum_i \frac{\Delta s_i}{v_i},
$$

$$
T_{\text{trap}} = \sum_i \frac{\Delta s_i}{2}\left(\frac{1}{v_i} + \frac{1}{v_{i+1}}\right),
$$

and, when uniform spacing and an even number of panels are available,

$$
T_{\text{Simpson}} = \frac{\Delta s}{3}\left[f_0 + f_N + 4\sum_{j\,\text{odd}} f_j + 2\sum_{j\,\text{even}} f_j\right],
\qquad f_j = \frac{1}{v_j}.
$$

The implementation also reports a kinematic segment estimate,

$$
T_{\text{kin}} = \sum_i \frac{2\Delta s_i}{v_i + v_{i+1}},
$$

which corresponds to assuming piecewise-constant longitudinal acceleration over each segment.

**Expected outcome.**
All methods should produce lap times of similar order. If the discretization is sufficiently refined, the spread between methods should be modest.

**How to analyze it.**
This figure turns the lap-time calculation into a numerical-method comparison. Agreement among the bars indicates that the discretization is fine enough for time integration; wide separation suggests either insufficient spatial resolution or strong local speed variation that is under-resolved.

## 4. Governing Dynamic Constraints

The speed analysis rests on three core constraints.

### 4.1 Lateral Constraint

The lateral acceleration requirement is

$$
a_y = v^2|\kappa|.
$$

This imposes a curvature-dependent speed ceiling.

### 4.2 Longitudinal Propagation Constraint

The discrete propagation law is

$$
v_{i+1}^2 = v_i^2 + 2a_{x,i}\Delta s_i.
$$

The forward pass uses a positive acceleration bound associated with engine capability, whereas the backward pass uses the braking limit.

### 4.3 Friction-Circle Constraint

The combined tire-force limit is modeled through

$$
a_x^2 + a_y^2 \le a_{\text{fric}}^2,
$$

with

$$
a_{\text{fric}} = \mu g
$$

under the simplified assumption of constant total normal load. This relation ensures that strong cornering reduces the longitudinal acceleration or braking that can be applied simultaneously.

## 5. Recommended Interpretation of the Workflow

The four commands should be interpreted as a structured numerical study rather than as independent utilities.

1. `list-tracks` identifies the available experimental cases.
2. `audit-track` checks whether the raw data are numerically trustworthy.
3. `compare-methods` quantifies the influence of discretization on curvature.
4. `analyze-track` transforms the validated geometry into a standing-start performance estimate.

In this sense, the workflow mirrors a sound numerical-analysis sequence: inspect the raw data, test the differentiation scheme, propagate the dynamic state, and then compare integration formulas for the final performance metric.

## 6. Concluding Technical Remarks

The current command set is sufficient to support a coherent technical narrative for the centerline baseline study. First, the geometry audit confirms whether the raw dataset is suitable for numerical differentiation. Second, the curvature comparison quantifies whether resampling materially changes the derivative estimate. Third, the speed analysis converts that geometry into interpretable dynamic quantities: speed, acceleration, braking phases, and lap time. Finally, the integration comparison exposes the numerical sensitivity of the terminal performance metric.

From a report-writing perspective, this makes the current toolchain appropriate for an academic discussion centered on numerical methods: finite differences for curvature, interpolation and resampling in arc length, discrete kinematic propagation, constrained speed limiting, and quadrature-based lap-time estimation.