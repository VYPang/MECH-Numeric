# 2026_05_22 Web UI and Visualization Report

## 1. Purpose and Scope

This report documents the current local web interface built for the centerline-based racing trajectory study. Its purpose is to describe the web-facing workflow, the backend data flow, the structure and interpretation of the visualizations, and the numerical meaning of the metrics exposed by the dashboard. The report is not primarily a software-engineering note; instead, it records how the existing centerline analysis pipeline has been surfaced through a browser-based interface so that track-level comparisons and figure inspection can be performed interactively.

The present implementation uses a FastAPI backend in `webapp/`, a static frontend in `frontend/`, and Plotly for client-side visualization. The numerical core remains the existing analysis stack in `source/`: track loading, geometry audit, curvature resampling, and three-pass speed-profile propagation are reused rather than duplicated.

At this stage, the web interface is method-aware but only one method is active:

1. `centerline_baseline`

The interface is structured so that a future `trajectory_optimized` method can be added without changing the fundamental API contract or page layout.

## 2. Update Summary

The web UI introduces a local interactive analysis workflow that complements the command-line tools. Relative to the earlier CLI-only workflow, the key additions are:

1. a browser-accessible overview page for all tracks,
2. a track-detail page with linked Plotly visualizations,
3. a FastAPI backend that serializes baseline results into plot-ready arrays,
4. a cached, method-aware service layer that wraps the existing solver, and
5. derived geometry and performance metrics for ranking and filtering.

The track map has also been enhanced beyond a plain centerline plot. The current visualization includes left and right boundaries reconstructed from the stored track widths, a speed-colored reference trajectory, acceleration and braking event markers, a labeled start point, and a direction arrow indicating the running direction. The map frame is rendered as a square and constrained to a centered narrow panel so that zooming is easier and the spatial view does not dominate the full page width.

The current difficulty score should be interpreted carefully. It is implemented as a normalized heuristic ranking signal for exploratory sorting, not as a validated physical scalar measure of track difficulty. The dashboard keeps the score visible for quick comparison, but the underlying component metrics remain exposed so that the interpretation does not collapse onto a single opaque number.

## 3. System-Level Technical Summary

### 3.1 Backend Architecture

**Purpose.**
The backend converts the existing centerline-baseline computation into browser-consumable JSON payloads. Its role is to preserve the validated numerical pipeline while separating analysis from presentation.

**Technical structure.**
The main application is created in `webapp/main.py`. A single `AnalysisService` instance is attached to FastAPI application state. That service:

1. loads the vehicle configuration,
2. loads a selected track dataset,
3. runs the geometry audit,
4. computes the resampled curvature profile,
5. computes the three-pass speed solution, and
6. packages geometry metrics, performance metrics, residuals, and plot arrays.

The service caches results by `(track, method)` pair so repeated visits to the same track do not recompute the same baseline solution unnecessarily.

**Expected outcome.**
The browser can request any track summary or detail payload without directly invoking the numerical code. The backend remains the single source of truth for both the dashboard and any future comparison views.

### 3.2 Frontend Architecture

**Purpose.**
The frontend provides a lightweight local interface for track browsing and detail inspection without introducing a heavier JavaScript framework.

**Technical structure.**
The frontend consists of `index.html`, `track.html`, `styles.css`, and `app.js`. Plotly is loaded from a CDN and all state is fetched from the backend through REST endpoints. The overview page renders a sortable ranking table. The detail page renders multiple linked figures and numerical summary cards for a single track.

**Expected outcome.**
The user can inspect track behavior spatially and parametrically within the same page, while preserving a simple local deployment model.

### 3.3 Validation Against the CLI Baseline

**Purpose.**
To ensure that the web interface is a visualization layer over the same underlying analysis, rather than a second, diverging implementation.

**Validation result.**
The FastAPI output for Monza was checked against the command-line `analyze-track` result. The trapezoidal lap-time estimate and residual values matched the CLI output, which confirms that the web payloads are consistent with the baseline solver.

**Technical interpretation.**
This matters because the dashboard is intended to support interpretation, not alter the numerical method. Matching CLI results indicate that the browser interface is reading the same computed state that the command-line workflow already trusts.

## 4. Route-Level Technical Summary

### 4.1 `/`

**Purpose.**
The root page presents the full-track overview and ranking interface.

**Expected outcome.**
The user receives a sortable table of all tracks, along with headline geometry and performance metrics. The page also displays the currently loaded vehicle parameters so that the ranking context is explicit.

**Technical interpretation.**
This page is the dashboard entry point. It is intended for cross-track comparison rather than detailed per-track inspection.

### 4.2 `/track?name=<track>`

**Purpose.**
The track detail page provides a focused diagnostic view for one track.

**Expected outcome.**
The user receives metric cards, a spatial track map, a speed profile, a longitudinal-acceleration plot, a curvature plot, and a numerics panel containing integration results and residual checks.

**Technical interpretation.**
This page exposes the same quantities that appear in the CLI workflow, but in a form that supports coordinated spatial and parametric inspection.

### 4.3 `/api/tracks`

**Purpose.**
To expose the list of valid track names to the frontend.

**Expected outcome.**
The browser receives a JSON array of all available track datasets.

**Technical interpretation.**
This route defines the set of valid problem instances for the interface, analogous to the CLI `list-tracks` command.

### 4.4 `/api/rankings`

**Purpose.**
To provide a sortable cross-track metric table for the overview dashboard.

**Expected outcome.**
The browser receives one row per analyzed track, including geometry metrics, performance metrics, and a current heuristic difficulty score.

**Technical interpretation.**
This route converts many individual track analyses into a comparative ranking surface. It is the main aggregation endpoint in the current web workflow.

### 4.5 `/api/tracks/{track}/summary`

**Purpose.**
To expose compact per-track information without transmitting every plot array.

**Expected outcome.**
The browser receives audit statistics, integration results, residuals, and track metrics.

**Technical interpretation.**
This route provides a concise summary layer suitable for future comparison cards, lightweight previews, or secondary panels.

### 4.6 `/api/tracks/{track}/baseline`

**Purpose.**
To supply the detail page with all plot-ready arrays for the baseline method.

**Expected outcome.**
The browser receives centerline coordinates, resampled path coordinates, curvature, arc-length coordinates, speed-cap data, forward-pass data, final speed, longitudinal acceleration, lateral acceleration, integration estimates, and residuals.

**Technical interpretation.**
This is the route that materializes the full numerical state of the baseline analysis into a visualization-ready payload.

## 5. Metric-Level Technical Summary

### 5.1 Geometry Metrics

**Purpose.**
Geometry metrics summarize the track shape independently of the vehicle model.

**Included quantities.**
The current implementation exposes total lap length, closure gap, spacing coefficient of variation, mean absolute curvature, maximum absolute curvature, curvature RMS, corner severity index, mean track width, and minimum track width.

**Technical interpretation.**
These quantities describe the shape regularity, path closure quality, and curvature load of the reference centerline. They are intended to remain meaningful even when the dynamic model is changed later.

The curvature RMS is computed from the resampled curvature trace as

$$
\kappa_{\mathrm{rms}}
=
\sqrt{\frac{1}{N}\sum_{i=1}^{N}\kappa_i^2}.
$$

The corner severity index is constructed from the upper quartile of absolute curvature values. If $|\kappa|_{(0.75)}$ denotes the 75th-percentile threshold, then

$$
C_{\mathrm{sev}}
=
\frac{1}{N}
\sum_{i:|\kappa_i|\ge |\kappa|_{(0.75)}}
|\kappa_i|.
$$

This is intended to emphasize the tightest cornering regions rather than only the global mean curvature.

### 5.2 Performance Metrics

**Purpose.**
Performance metrics summarize the solved baseline speed profile and the implied control phases.

**Included quantities.**
The current implementation exposes lap time, average speed, maximum speed, minimum speed, speed standard deviation, mean absolute longitudinal acceleration, acceleration fraction, braking fraction, coasting fraction, phase-transition count, and fraction of samples near the lateral limit.

**Technical interpretation.**
These quantities are vehicle-dependent because they are derived from the solved speed profile rather than from geometry alone.

The average speed is computed from

$$
\bar{v} = \frac{L}{T},
$$

where $L$ is the total lap length and $T$ is the trapezoidal lap-time estimate. The mean absolute longitudinal acceleration is

$$
\overline{|a_x|} = \frac{1}{N}\sum_{i=1}^{N}|a_{x,i}|.
$$

The lateral-limit fraction counts the share of samples satisfying

$$
a_{y,i} \ge 0.95\,a_{y,\max}.
$$

This gives a simple indicator of how often the lap operates close to the lateral capability ceiling.

### 5.3 Difficulty Metric

**Purpose.**
The difficulty score currently serves only as an exploratory ranking aid on the overview page.

**Current implementation.**
Each selected component is min-max normalized across the available track set, then combined linearly:

$$
S
=
0.30\,\kappa_{\mathrm{rms}}
+ 0.25\,C_{\mathrm{sev}}
+ 0.15\,\sigma_v
+ 0.15\,\overline{|a_x|}
+ 0.15\,f_{\mathrm{lat}}.
$$

**Technical interpretation.**
This scalar is not a validated physical model of difficulty. It is a provisional, normalized composite used to support fast sorting in the dashboard. The component metrics are therefore more important than the scalar itself, and any formal use of a single difficulty number would require calibration or validation against an external target.

## 6. Figure-by-Figure Interpretation

### 6.1 Overview Ranking Table

**Purpose.**
To compare all available tracks on one page using a consistent baseline solver and a shared set of summary metrics.

**How it is constructed mathematically.**
Each row aggregates the geometry and performance metrics derived from the resampled centerline and the baseline speed profile. The table can then be sorted by any displayed quantity.

**Expected outcome.**
Tracks with longer lap times, larger speed variation, higher curvature load, or larger longitudinal workload rise toward the top depending on the active sorting column.

**How to analyze it.**
This table is the browser equivalent of a multi-case batch study. It is most useful for identifying which tracks deserve closer visual inspection on the detail page.

### 6.2 Metric Cards

**Purpose.**
To provide a compact track summary before the user studies the plots.

**How it is constructed mathematically.**
Each card is a direct scalar extracted from the solved geometry or speed profile.

**Expected outcome.**
The cards summarize the key dynamic and geometric scales of the selected track at a glance.

**How to analyze it.**
The cards provide context for the plots that follow. They are especially useful when comparing two tracks manually across multiple browser tabs.

### 6.3 Track Map and Speed Envelope

**Purpose.**
To provide a spatial view of the baseline solution around the lap.

**How it is plotted mathematically.**
The resampled reference trajectory is plotted in the $(x,y)$ plane and colored by the solved final speed. The left and right boundaries are reconstructed from the centerline and the stored widths using a local unit normal:

$$
\mathbf{t}_i
\approx
\frac{\mathbf{r}_{i+1}-\mathbf{r}_{i-1}}{\lVert \mathbf{r}_{i+1}-\mathbf{r}_{i-1} \rVert},
\qquad
\mathbf{n}_i = (-t_{y,i},\, t_{x,i}),
$$

$$
\mathbf{r}_{L,i} = \mathbf{r}_i + w_{L,i}\mathbf{n}_i,
\qquad
\mathbf{r}_{R,i} = \mathbf{r}_i - w_{R,i}\mathbf{n}_i.
$$

Acceleration and braking events are marked when the longitudinal-acceleration state changes into accelerating or braking. The start point is labeled explicitly, and a direction arrow is drawn from the initial path heading.

**Expected outcome.**
Fast straights appear in warm colors and slow corners in cool colors. The spatial positions of braking and acceleration onset become immediately visible. The boundaries show whether the stored width channels are geometrically consistent with the displayed centerline.

**How to analyze it.**
This is the main interpretive plot for spatial dynamics. It answers where the vehicle is fast, where it brakes, where it begins to accelerate again, and how those events align with the geometric corridor.

### 6.4 Speed Profile

**Purpose.**
To compare the local lateral speed cap, the forward pass, and the final baseline solution as functions of arc length.

**How it is plotted mathematically.**
The lateral limit is based on

$$
a_y = v^2|\kappa|,
$$

so the speed cap is

$$
v_{\mathrm{lat},i}
=
\sqrt{\frac{a_{y,\mathrm{lim}}}{|\kappa_i|+\varepsilon}}.
$$

The forward pass and final pass are derived from the discrete propagation law

$$
v_{i+1}^2 = v_i^2 + 2a_{x,i}\Delta s_i,
$$

subject to the drive, braking, and friction-circle limits inherited from the baseline solver.

**Expected outcome.**
The final speed profile lies below any infeasible forward-only or lateral-only envelope when braking feasibility or combined-tire-force effects become active.

**How to analyze it.**
This plot shows exactly where the lap is limited by curvature, propulsion, or braking propagation.

### 6.5 Longitudinal Acceleration Plot

**Purpose.**
To show how the solved speed profile decomposes into accelerating, coasting, and braking phases.

**How it is plotted mathematically.**
At each segment midpoint,

$$
a_{x,i} = \frac{v_{i+1}^2 - v_i^2}{2\Delta s_i}.
$$

The plot also shows the forward longitudinal limit and the braking limit as reference envelopes.

**Expected outcome.**
Positive regions align with acceleration zones, negative regions align with braking zones, and near-zero regions indicate coasting or locally speed-capped behavior.

**How to analyze it.**
This is the clearest plot for distinguishing drive-limited and brake-limited segments of the lap.

### 6.6 Curvature Plot

**Purpose.**
To expose the resampled curvature field directly as a function of arc length.

**How it is plotted mathematically.**
The figure displays the resampled curvature $\kappa(s)$ from the central-difference derivative procedure already used by the baseline solver.

**Expected outcome.**
Tight corners appear as sharp curvature peaks, while long straights appear near zero curvature.

**How to analyze it.**
This plot is the geometric companion to the speed profile. Regions of large $|\kappa|$ should align with speed reductions and braking propagation.

### 6.7 Numerics Panel

**Purpose.**
To expose the underlying integration estimates and residual checks used to assess the numerical consistency of the solution.

**How it is presented mathematically.**
The panel reports the same lap-time estimates and constraint residuals that the CLI pipeline prints numerically.

**Expected outcome.**
The different integration formulas should remain close in value, and the residuals should stay near zero when the propagated solution is internally consistent.

**How to analyze it.**
This panel is the browser equivalent of a numerical sanity check. It helps distinguish a visually plausible plot from a quantitatively trustworthy result.

## 7. Interactive Behavior

The current detail page supports linked hover between the parametric plots and the spatial map. When the user hovers over the speed profile, longitudinal-acceleration plot, or curvature plot, the nearest point on the track map is highlighted. This interaction allows the user to connect a one-dimensional arc-length event with its physical location on the circuit.

This is important because many dynamic features are easier to detect in one representation than the other. For example, the speed plot may show a sharp drop clearly, while the map identifies whether that drop occurs in a hairpin, a chicane, or a medium-speed bend.

## 8. Recommended Interpretation of the Web Workflow

The web interface should be understood as a layered visualization workflow built on top of the same analysis sequence already established in the CLI tools.

1. The overview page identifies cross-track patterns and outliers.
2. The detail page translates a selected baseline solution into coordinated visual diagnostics.
3. The numerics panel verifies that the rendered plots remain consistent with the underlying integration and constraint checks.

In this sense, the web UI does not replace the numerical pipeline. It reorganizes the outputs so that comparative analysis, figure inspection, and interpretation become faster and more interactive.

## 9. Concluding Technical Remarks

The current web UI is sufficient to support a coherent interactive baseline study. It exposes the same centerline-based solver through a browser, adds cross-track comparison through a sortable overview table, and provides spatially and parametrically linked plots for a selected track. The backend remains method-aware, so future optimized-trajectory results can be integrated without changing the overall structure of the interface.

From a report-writing perspective, the web layer now supports a more visual presentation of the same numerical ideas already present in the CLI workflow: resampled geometry, curvature-sensitive speed limits, three-pass speed propagation, longitudinal phase identification, and quadrature-based lap-time comparison. The main caveat is that the current difficulty score should not be overinterpreted. The more defensible outputs remain the explicit geometry metrics, the explicit performance metrics, and the underlying baseline figures themselves.