# Three-Pass Centerline Speed Analysis Proposal and Implementation Plan

## 1. Project Scope

This stage of the project will reproduce the **fixed-path speed-profile computation** from *A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories*, while replacing the original high-fidelity vehicle model with a simpler point-mass model.

The current goal is **not** to optimize the racing line yet. Instead, we first analyze the closed-circuit track centerline from the supplied racing-track dataset and compute a physically feasible **first-lap** speed profile using a three-pass method:

1. curvature-based lateral speed limit,
2. forward acceleration integration,
3. backward braking integration.

This centerline result will become the baseline for later comparison against optimized racing lines and the provided reference racelines.

## 2. Main Research Question

Given a closed two-dimensional racing track represented by centerline coordinates, how can we compute a feasible standing-start first-lap speed profile using a simplified three-pass method, and what track properties can be extracted from the resulting curvature, velocity, acceleration, braking, and lap-time profiles?

This stage focuses on preliminary fixed-path analysis. The major outcome is not a final optimized racing line, but a systematic numerical framework for understanding how the geometry of each track affects speed feasibility.

## 3. Relation to the Paper

The original paper uses a sequential two-step structure:

1. compute the fastest feasible speed profile for a fixed path,
2. update the path using a vehicle-dynamics-based optimization step.

This project stage recreates the first step only. The paper's detailed tire and bicycle-model physics are replaced by a simpler model based on path curvature, lateral acceleration, longitudinal acceleration, and braking limits.

Therefore, the project should be described as replicating the **numerical structure** of the three-pass method rather than the complete vehicle dynamics of the paper.

## 4. Dataset and Geometry Input

The dataset provides one CSV file per track in `data/tracks/`. Each track file has the format

```text
x_m, y_m, w_tr_right_m, w_tr_left_m
```

where:

- `x_m` and `y_m` are centerline coordinates in meters,
- `w_tr_right_m` is the track width to the right of the centerline,
- `w_tr_left_m` is the track width to the left of the centerline.

In code, one track will be stored as sampled discrete vectors such as

$$
x = [x_0, x_1, \dots, x_{N-1}],
\qquad
y = [y_0, y_1, \dots, y_{N-1}],
$$

and similarly for left width, right width, segment length, curvature, and speed:

$$
\Delta s = [\Delta s_0, \Delta s_1, \dots, \Delta s_{N-1}],
\qquad
\kappa = [\kappa_0, \kappa_1, \dots, \kappa_{N-1}],
$$

$$
v = [v_0, v_1, \dots, v_{N-1}].
$$

Here, index \(i\) means the \(i\)-th sampled location along the closed track. So when the document later refers to "discrete arrays," it simply means these sampled vectors of track or state values.

The initial implementation will operate on one selected track, but the file-loading and analysis pipeline must be general enough to run on all tracks later.

## 5. Standing-Start Lap Interpretation

In this project stage, the centerline analysis will be treated as a standing-start lap rather than a steady flying lap. This means the chosen start point is physically meaningful, and the vehicle begins from rest at that location. If the sampled track points are stored as

$$
(x_0,y_0), (x_1,y_1), \dots, (x_{N-1},y_{N-1}),
$$

then index \(0\) is taken to be the actual lap start used in the computation, and the boundary condition is

$$
v_0 = 0.
$$

The first pass still computes the curvature-based speed cap at every sampled point on the track. That step does not depend on the initial speed; it only describes the maximum locally feasible speed allowed by the path geometry. The forward pass then starts from the chosen start point and accelerates according to the maximum longitudinal acceleration while remaining below the local speed cap. The backward pass is applied from the end of the sampled lap back toward the beginning so that each point also satisfies the braking requirement imposed by later corners.

Under this interpretation, the track geometry is still closed, but the speed profile is not treated as periodic. The vehicle starts from rest at the selected start point and finishes the lap with whatever speed is physically achievable by the end of the computation. In other words, there is no requirement that the speed at the end of the lap must match the speed at the beginning. This makes the standing-start formulation appropriate for modeling the first lap from a chosen launch point rather than a fully settled racing lap.

In implementation, the sampled vectors for \(x\), \(y\), \(\Delta s\), \(\kappa\), width, and speed cap can remain in their original file order as long as that order matches the intended lap direction. No cyclic reindexing is needed. The selected first point is simply treated as the physical start of the lap, the speed is initialized with \(v_0=0\), and the three-pass procedure is carried out along the sampled track in that fixed order.

## 6. Numerical Geometry Analysis

Before deciding whether resampling is necessary, the first script should audit and visualize the raw dataset. The audit should report:

- number of centerline points,
- total centerline length,
- distance between the first and last point,
- minimum, maximum, mean, and standard deviation of segment length,
- left and right width statistics,
- potential outlier segment lengths,
- whether the sampling is close to uniform.

For consecutive centerline points

$$
p_i = (x_i, y_i),
$$

the segment length is

$$
\Delta s_i = \sqrt{(x_{i+1}-x_i)^2 + (y_{i+1}-y_i)^2}.
$$

The cumulative arc length is

$$
s_0 = 0,
\qquad
s_{i+1}=s_i+\Delta s_i.
$$

The first milestone should visualize the raw centerline, track width, and segment-length distribution before applying smoothing or resampling.

## 7. Curvature Computation

For a continuous path

$$
p(s) = (x(s), y(s)),
$$

the curvature is

$$
\kappa(s)=
\frac{x'(s)y''(s)-y'(s)x''(s)}{\left(x'(s)^2+y'(s)^2\right)^{3/2}}.
$$

In a numerical implementation, the derivatives must be approximated from discrete data. If the points are approximately uniformly spaced in arc length, central differences can be used:

$$
x'_i \approx \frac{x_{i+1}-x_{i-1}}{2\Delta s},
\qquad
y'_i \approx \frac{y_{i+1}-y_{i-1}}{2\Delta s},
$$

and

$$
x''_i \approx \frac{x_{i+1}-2x_i+x_{i-1}}{\Delta s^2},
\qquad
y''_i \approx \frac{y_{i+1}-2y_i+y_{i-1}}{\Delta s^2}.
$$

Because the track is closed, the indexing should be periodic:

$$
x_{-1}=x_{N-1},
\qquad
x_N=x_0,
$$

and similarly for \(y\).

This periodic indexing is used only to evaluate the geometry of a closed path. It does not mean the speed boundary condition is periodic. In the present formulation, the geometry is closed but the speed solve is still a standing-start first-lap computation with \(v_0=0\).

If the raw data are not uniformly spaced, then central differences on the raw index may produce inaccurate curvature. In that case, the next stage should resample the path by arc length or use spline-based derivatives. This provides a natural numerical-method comparison:

- raw finite differences,
- arc-length-resampled finite differences,
- spline derivative curvature.

## 8. Simplified Vehicle Model

The vehicle is modeled as a point mass moving tangent to the chosen path. The main variables are:

- path position \(s\),
- speed \(v(s)\),
- curvature \(\kappa(s)\),
- longitudinal acceleration \(a_x(s)\),
- elapsed time \(t(s)\).

The model parameters should be stored in JSON configuration files under `config/`, so they can be modified when the teammate's model is integrated.

Suggested files:

```text
config/
  vehicle.json
  analysis.json
```

The vehicle configuration should include:

```json
{
  "ay_max_mps2": 13.7,
   "ax_engine_max_mps2": 4.0,
  "brake_max_mps2": 8.0,
   "mu": 1.4,
   "F_z_n": 7848.0,
  "v_max_mps": 90.0,
  "curvature_epsilon": 1e-6,
  "v_min_mps": 1.0
}
```

These values are preliminary and should be treated as tunable assumptions, not as a final validated vehicle model.

## 9. Three-Pass Speed Method

### 9.1 Pass 1: Lateral Speed Limit

The lateral acceleration required to travel at speed \(v\) through curvature \(\kappa\) is

$$
a_y = v^2 |\kappa|.
$$

Given a maximum lateral acceleration \(a_{y,\max}\), the lateral speed limit is

$$
v_{lat,i} = \sqrt{\frac{a_{y,\max}}{|\kappa_i|+\epsilon}}.
$$

An optional top-speed limit can be applied:

$$
v_{cap,i}=\min(v_{lat,i},v_{\max}).
$$

### 9.2 Pass 2: Forward Acceleration Integration

The longitudinal speed propagation equation is

$$
\frac{d(v^2)}{ds}=2a_x.
$$

Using maximum acceleration \(a_{x,\max}\), the forward update is

$$
v^f_0 = 0,
$$

and for \(i=0,1,\dots,N-2\),

$$
v^f_{i+1}
=
\min\left(
v_{cap,i+1},
\sqrt{(v^f_i)^2 + 2a_{x,\max}\Delta s_i}
\right).
$$

This pass prevents the vehicle from instantly accelerating to the curvature-based speed cap and enforces the standing-start boundary condition at the chosen lap start.

### 9.3 Pass 3: Backward Braking Integration

Using maximum braking magnitude \(b_{\max}\), the backward update is

$$
v_i
=
\min\left(
v^f_i,
\sqrt{v_{i+1}^2 + 2b_{\max}\Delta s_i}
\right).
$$

This pass propagates future corner speed requirements backward and determines where braking must begin.

## 10. Lap Time and Acceleration Profiles

After computing the final feasible speed profile, lap time is approximated by numerical integration:

$$
T \approx \sum_{i=0}^{N-1}\frac{\Delta s_i}{\max(v_i,v_{\min})}.
$$

The discrete longitudinal acceleration can be estimated from

$$
a_{x,i}
=
\frac{v_{i+1}^2-v_i^2}{2\Delta s_i}.
$$

The lateral acceleration is

$$
a_{y,i}=v_i^2|\kappa_i|.
$$

These profiles allow the analysis to identify acceleration zones, braking zones, and curvature-limited zones.

## 11. Constraint Residual Checks

The computed speed profile should be checked using residuals. For lateral acceleration:

$$
r_{lat}=\max_i\left(v_i^2|\kappa_i|-a_{y,\max}\right).
$$

For forward acceleration:

$$
r_{acc}=\max_i\left(v_{i+1}^2-v_i^2-2a_{x,\max}\Delta s_i\right).
$$

For braking:

$$
r_{brake}=\max_i\left(v_i^2-v_{i+1}^2-2b_{\max}\Delta s_i\right).
$$

A valid speed profile should have residuals less than a chosen numerical tolerance, allowing for floating-point roundoff.

## 12. Friction-Circle Extension

The first milestone should use the simpler uncoupled three-pass method. A later refinement can add a friction-circle constraint.

The intuition is that the tire has a limited total friction capacity. If the vehicle is using much of this capacity for turning, less remains for accelerating or braking.

A simplified friction-circle model is

$$
\left(\frac{a_x}{a_{x,\max}}\right)^2
+
\left(\frac{a_y}{a_{y,\max}}\right)^2
\le 1.
$$

Since

$$
a_y=v^2|\kappa|,
$$

the available longitudinal acceleration would decrease in high-curvature sections. This extension should be implemented only after the baseline method is working.

## 13. Numerical-Methods Analysis Before Trajectory Optimization

Before moving to trajectory optimization, the centerline study should already include explicit numerical-method analysis in the spirit recommended by the course lecturer. The purpose is not only to produce one lap-time number, but to justify why a chosen numerical method is trustworthy, how it behaves under refinement, and when a seemingly good answer may be misleading.

The first analysis point is **track representation and resampling**. The raw dataset should first be audited directly, but the project should then compare at least two geometry treatments, such as raw point-to-point evaluation and arc-length-resampled evaluation. If time permits, spline-based interpolation can be added as a third method. The main questions are whether the centerline remains smooth, whether curvature becomes more stable after resampling, and whether a visually smooth path can still produce numerically unstable derivatives.

The second analysis point is **numerical differentiation for curvature**. Since curvature depends on first and second derivatives, it is one of the most error-sensitive quantities in the whole pipeline. At minimum, the project should compare raw central differences against central differences after arc-length resampling. A stronger version should also compare spline-based derivative evaluation. The validation should not rely only on track data, because the true curvature is unknown there. Instead, the code should also be tested on simple reference curves such as a straight line, a circle, and a sinusoidal path where the expected geometric behavior is known. The main outcome should be a reasoned choice of curvature method based on stability, noise sensitivity, and refinement behavior.

The third analysis point is **numerical integration for arc length and lap time**. This is the place where the lecturer's warning is especially relevant: a low-order method such as the trapezoidal rule may occasionally appear accurate because local errors cancel, but that alone is not a good justification. The project should compare composite trapezoidal integration, Simpson's 1/3 rule when applicable, and a mixed or segmented strategy when the track data are not evenly suited to a single rule. One useful idea is to segment the track into lower-curvature and higher-curvature regions and check whether a mixed strategy can achieve similar accuracy at lower cost. Reference values should come either from analytical test curves or from a much finer discretization. The report should explicitly discuss the tradeoff between computational cost and integration accuracy rather than selecting a method only because it produces one favorable result.

The fourth analysis point is **speed propagation as a numerical IVP-like computation**. In the current proposal, the main method uses the direct update of \(v^2\),

$$
v_{i+1}^2 = v_i^2 + 2a_x\Delta s_i,
$$

which is simple and physically interpretable. However, the project should still analyze why this update is preferable for the current problem. A useful comparison is against an explicit Euler discretization of

$$
\frac{dv}{ds} = \frac{a_x}{v},
$$

with optional comparison to a higher-order RK method only if needed. The analysis questions are whether the \(v^2\) formulation is more stable near low speed, how discretization size affects predicted braking locations, and whether higher-order IVP solvers provide any practical benefit when the acceleration model is piecewise constrained rather than smooth.

The fifth analysis point is **event detection and root-finding-based post-processing**. Even before path optimization, the centerline solution naturally produces important events such as local curvature peaks, minimum-speed points, braking onset points, and transitions between acceleration-limited and braking-limited regions. These events can be located more precisely by defining scalar functions and applying root-finding or bracketing methods. For example, one may detect braking onset by locating where the forward-limited and backward-limited speed envelopes intersect. This gives a natural place to include bisection, secant, or Newton-type analysis, with the main focus on robustness versus efficiency.

The sixth analysis point is **grid refinement and sensitivity analysis**. The project should not trust one discretization level blindly. Instead, it should rerun the same centerline analysis using several sampling resolutions or resampling spacings and observe the effect on curvature, lap time, braking-point location, and constraint residuals. This is important because a visually acceptable plot can still hide a numerically poor derivative or integration result. The sensitivity study should also vary key vehicle parameters such as \(a_{y,\max}\), \(a_{x,\max}\), and \(b_{\max}\) so that the final report can distinguish geometric effects from parameter effects.

Taken together, these comparisons give the centerline stage its numerical-methods identity. Even before trajectory optimization is introduced, the project can already answer course-relevant questions such as which differentiation method is stable, which integration method is justified, how discretization affects the result, and whether a more sophisticated method actually improves the computation enough to be worth its additional cost.

## 14. Metrics for Track Analysis

For each analyzed track, the program should compute:

- total centerline length,
- estimated lap time,
- average speed,
- minimum and maximum speed,
- maximum curvature,
- mean absolute curvature,
- 95th percentile absolute curvature,
- curvature energy,
- maximum lateral acceleration,
- maximum acceleration demand,
- maximum braking demand,
- percentage of track that is curvature-limited,
- percentage of track that is acceleration-limited,
- percentage of track that is braking-limited,
- number and location of major braking zones.

Curvature energy can be defined as

$$
E_{\kappa}=\sum_i \kappa_i^2\Delta s_i.
$$

This is not physical energy. It is a geometry metric that measures how much turning demand exists along the track.

## 15. Visualization Plan

The analysis should generate both static and interactive plots.

### 14.1 Dataset Audit Plots

- centerline with approximate track boundaries,
- segment length versus point index,
- histogram of segment lengths,
- left and right track width versus arc length.

### 14.2 Three-Pass Analysis Plots

- centerline colored by speed,
- centerline colored by curvature,
- speed versus arc length,
- curvature versus arc length,
- longitudinal acceleration versus arc length,
- lateral acceleration versus arc length,
- active constraint versus arc length.

### 14.3 Heatmaps

Useful heatmaps for the first milestone include:

1. **parameter-sensitivity heatmap**  
   Sweep two vehicle parameters, such as \(a_{y,\max}\) and \(b_{\max}\), and color by lap time.

2. **track-metric heatmap**  
   Rows are tracks and columns are metrics such as lap time, length, maximum curvature, braking fraction, and acceleration fraction.

3. **normalized-position heatmap**  
   Rows are tracks and columns are normalized lap position \(s/L\), with color showing speed, curvature, or active constraint.

Later, after path optimization is implemented, another useful heatmap will be:

4. **optimization-change heatmap**  
   Rows are optimization iterations and columns are normalized lap position, with color showing changes in speed, curvature, or local time loss.

In addition to the plots above, the centerline stage should include method-comparison visualizations such as curvature profiles from multiple differentiation methods on the same track, lap-time convergence versus grid spacing, and integration-error comparisons against a finer reference solution. Those plots will make the numerical tradeoffs visible rather than leaving them only as text in the report.

## 16. Software Structure

The source code should be placed in `source/`. A possible structure is:

```text
source/
  cli.py
  config.py
  data_loader.py
  geometry.py
  curvature.py
  speed_profile.py
  metrics.py
  plots.py
  sensitivity.py
config/
  vehicle.json
  analysis.json
outputs/
  figures/
  html/
  tables/
```

The project should use:

- Typer for command-line interaction,
- Rich for progress bars, status messages, and summary tables,
- Matplotlib for report-ready 2D plots,
- Plotly for interactive HTML visualizations.

## 17. Proposed CLI Commands

The following commands are recommended:

```bash
python -m source.cli audit-track Monza
python -m source.cli analyze-track Monza
python -m source.cli compare-methods Monza
python -m source.cli compare-tracks
python -m source.cli sensitivity Monza --x ay_max_mps2 --y brake_max_mps2
```

The first implementation should prioritize `audit-track` and `analyze-track` for one selected track. The `compare-methods` command should be added before all-track comparison so that the chosen numerical methods are justified early rather than assumed.

## 18. Implementation Milestones

### Milestone 1: Dataset Audit

- Load one track CSV file.
- Verify columns and units.
- Compute segment lengths and cumulative arc length.
- Plot raw centerline and width information.
- Report spacing and closure statistics.

### Milestone 2: Geometry and Curvature Method Study

- Implement raw central-difference curvature.
- Implement at least one improved geometry treatment such as arc-length resampling.
- Plot curvature from multiple methods on the same track.
- Validate curvature behavior on reference curves such as a straight line and a circle.
- Decide whether resampling or spline-based differentiation is necessary.

### Milestone 3: Integration and Speed-Propagation Study

- Compare arc-length and lap-time integration methods.
- Implement the baseline \(v^2\) propagation method.
- Optionally compare with Euler propagation of \(v\) directly.
- Compute final speed and acceleration profiles.
- Check constraint residuals and sensitivity to discretization.

### Milestone 4: Centerline Three-Pass Solver and Event Analysis

- Compute lateral speed cap.
- Run forward acceleration pass.
- Run backward braking pass.
- Detect key events such as curvature peaks, minimum-speed points, and braking-onset points.
- Save the baseline centerline solution for later comparison.

### Milestone 5: Visualization and Metrics

- Generate 2D Matplotlib plots.
- Generate interactive Plotly HTML plots.
- Generate method-comparison plots and convergence plots.
- Compute summary metrics for the selected track.
- Save outputs under `outputs/`.

### Milestone 6: Generalization Across Tracks

- Run the same pipeline on all track files.
- Save one summary table for all tracks.
- Generate track-metric heatmaps.
- Identify tracks with high curvature demand, strong braking demand, or high speed potential.

### Milestone 7: Optional Reference Raceline Comparison

- Load the provided raceline file from `data/racelines/`.
- Run the same three-pass solver on the raceline.
- Compare centerline and raceline speed profiles.
- Report differences in length, curvature, speed, and lap time.

## 19. Expected Report Contribution

This part of the project should contribute four main results to the final report:

1. **Numerical-method explanation**  
   The report can explain how continuous path equations are converted into discrete finite-difference and integration formulas.

2. **Method-selection and tradeoff analysis**  
   The report can compare differentiation, integration, and propagation methods, and justify which methods are adopted based on accuracy, robustness, and cost rather than on isolated good-looking results.

3. **Centerline speed-feasibility analysis**  
   The report can show where the track geometry forces low speeds, where braking begins, and where acceleration is limited.

4. **Baseline for later optimization**  
   The centerline result provides a reference for measuring whether a future optimized path improves lap time, reduces curvature, or changes braking behavior.

## 20. Recommended First Track

The first test track should be one track with clear straights and corners. `Monza` is a good first choice because it has long straights and strong braking zones, making the forward and backward passes easy to interpret.

After Monza works, the same code can be tested on more complex tracks such as `Spa`, `Suzuka`, or `Silverstone`.

## 21. Summary

This proposal defines the first implementation stage as a generalizable three-pass centerline speed-analysis framework. The method uses simplified point-mass physics, preserves the core numerical idea of the paper's fixed-path speed computation, and applies it to a standing-start first-lap problem on a closed-circuit track.

The immediate deliverable is a command-line tool that can audit track geometry, compare numerical methods, compute curvature, run the three-pass speed solver, generate plots, and report track metrics. This provides a strong numerical-method foundation before moving to path-update optimization.