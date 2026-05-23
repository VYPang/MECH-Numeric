# 2026_05_23 Multimethod Web UI and Comparison Report

## 1. Purpose and Scope

This changelog records the next major extension of the local racing-trajectory dashboard after the baseline-only web UI described in the earlier `2026_05_22` report. The current update expands the browser workflow from a single centerline-baseline analysis into a persisted multimethod comparison environment with four selectable methods, startup-time cache reuse, and browser-based comparison views for both individual tracks and whole-track rankings.

The emphasis of this note is twofold:

1. to summarize the new numerical methods and backend behaviors now exposed through the web stack, and
2. to document the new comparison-oriented frontend interactions that were added on top of the precomputed cached results.

This report therefore covers the combined effect of the following changes:

1. integration of three additional method paths beyond the baseline,
2. persisted `web_cache` generation and reuse,
3. startup preloading with Rich progress reporting and explicit rerun control,
4. expanded per-method and cross-method ranking views,
5. comparison mode on the track-detail page, and
6. side-by-side and overlay spatial comparison visualizations.

## 2. Update Summary

Relative to the earlier web UI, the dashboard now supports the following methods:

1. `centerline_baseline`
2. `min_curvature` (A)
3. `min_lap_time` (B)
4. `min_curvature_custom` (C)

The backend no longer treats the browser as a thin wrapper over one on-demand baseline computation. Instead, it can precompute all supported `(track, method)` results, serialize them to `outputs/web_cache/`, and reload them on later launches if the cache signature remains valid.

The frontend has likewise moved from a single-method inspection workflow to a comparison workflow. The overview page now supports method-to-method ranking comparison through a rank-shift table and a difficulty-change scatter plot. The track-detail page now supports a dedicated compare mode, side-by-side method views, overlay method views, linked zoom/pan behavior, and per-method transparency control in overlay mode.

## 3. Numerical Method Integration

### 3.1 Baseline Method

The existing baseline remains the reference solution. It uses the centerline as the fixed path, computes a resampled curvature profile, solves the three-pass speed profile, and exposes the resulting geometry and performance metrics.

Its role in the new dashboard is now broader than before: it is both a standalone method and the default reference method for the new cross-method comparison pages.

### 3.2 Minimum-Curvature Method A

Method A is now fully integrated into the main source tree and web service layer as `min_curvature`. It uses the existing path-optimization workflow to solve for a curvature-reducing lateral offset within the track corridor, then recomputes curvature, speed, and derived metrics on the optimized path.

The regularization parameter for this method is now read from the runtime configuration rather than being treated as a hardcoded UI-side value.

### 3.3 Minimum-Lap-Time Method B

Method B is integrated as `min_lap_time`. It performs a direct finite-difference lap-time minimization and warm-starts from method A rather than from the centerline. This is important because B is intended to refine an already curvature-improved path rather than solve from scratch.

The implementation now includes explicit multiprocessing behavior for Linux, using a `fork` multiprocessing context on Ubuntu-class systems and a `spawn` fallback on macOS. Worker count is capped through configuration so runtime can be reduced without oversubscribing the machine.

### 3.4 Custom Minimum-Curvature Method C

Method C is integrated as `min_curvature_custom`. It is a custom-solver variant of the minimum-curvature problem and is intended to approximate the same optimization goal as method A while using a different internal numerical solution path.

In practical validation, C remained extremely close to A on tested tracks, which is consistent with the intended interpretation of C as a solver alternative rather than a different objective.

## 4. Backend and Cache Workflow Changes

### 4.1 Persisted Web Cache

The service layer now persists plot-ready JSON payloads and difficulty-score summaries under `outputs/web_cache/`. Each cached dataset is keyed by track name and method name, and the cache manifest records:

1. supported methods,
2. track list,
3. current vehicle configuration, and
4. cache version metadata.

This means the dashboard no longer needs to recompute a method every time the user changes a selector in the browser. If the manifest signature still matches the current configuration and supported-method set, the cached results are loaded directly.

### 4.2 Cache Invalidation Logic

The cache signature now depends on:

1. the full vehicle configuration,
2. the set of supported methods, and
3. the available track list.

This prevents stale cache reuse when the method set or optimization parameters are changed.

### 4.3 Startup Preload and Reuse

The web launcher in `script/run_webapp.py` now supports two startup behaviors:

1. reuse of a valid persisted cache, and
2. forced regeneration through `--rerun-results`.

When regeneration is needed, the launcher precomputes the entire `(track, method)` matrix and shows Rich progress feedback during the preload process.

## 5. Configuration Changes

The runtime configuration has been extended beyond the original vehicle parameters. It now includes optimization controls for the newly integrated methods, including:

1. method A regularization strength,
2. method B iteration count, finite-difference epsilon, line-search parameters, convergence tolerances, and worker count, and
3. method C APG/custom-solver iteration and tolerance settings.

These values are now centralized in `config/vehicle.json` and loaded through the typed configuration model in `source/config.py`.

## 6. Overview Page Changes

### 6.1 Per-Method Ranking Support

The overview page now supports direct method selection for the ranking table. Users can switch among baseline, A, B, and C without triggering recomputation as long as the persisted cache is valid.

### 6.2 Cross-Method Ranking Comparison

The overview page now includes a dedicated method-comparison section with:

1. reference-method and compare-method selectors,
2. a rank-shift table per track, and
3. a difficulty-comparison scatter plot.

The comparison table exposes cross-method deltas such as:

1. rank shift,
2. difficulty-score change,
3. lap-time change,
4. lap-time percentage change,
5. phase-transition change,
6. curvature change, and
7. lap-length change.

The difficulty scatter plot compares the reference difficulty score against the compare-method difficulty score and retains the rank-shift colorbar while suppressing redundant legend labels.

## 7. Track-Detail Page Changes

### 7.1 Compare Mode

The track-detail page now includes an explicit compare mode toggle. When compare mode is off, the page behaves as a single-method diagnostic page. When compare mode is on, the page switches into a dedicated cross-method comparison workspace.

This avoids forcing comparison content on users who only want to inspect one solution while still making comparison mode available without leaving the page.

### 7.2 Side-by-Side Comparison

Side-by-side comparison now renders the two compared methods in a left/right layout rather than stacking them vertically under normal desktop widths. The side-by-side panels are implemented to preserve a square spatial frame and to synchronize zoom/pan behavior across both maps.

The synchronized view now copies the actual final x/y axis ranges from the source plot after Plotly completes its aspect-ratio adjustments, so the compared panel tracks the same spatial window and center rather than merely replaying the requested relayout event.

### 7.3 Side-by-Side Map Rendering Style

The side-by-side spatial view now matches the individual map style rather than using a separate visual language. Each side includes:

1. left and right boundaries,
2. a speed-colored path representation,
3. acceleration labels `A`,
4. braking labels `B`,
5. a highlighted start point, and
6. the start-direction annotation.

The speed colormap range is shared with the individual mode so the same speed appears with the same saturation in both contexts.

### 7.4 Overlay Comparison Mode

Overlay mode has been redesigned into a compact comparison tool rather than a full-width duplicate of the side-by-side layout.

The overlay map now:

1. occupies roughly half the window width rather than spanning the full content width,
2. uses a single shared speed colorbar for both methods,
3. includes the same `A` and `B` event labels used by the individual map, and
4. exposes one opacity slider per compared method so the two trajectories can be faded independently.

This means the overlay view supports two different interpretive tasks:

1. velocity comparison through one shared colormap, and
2. geometric path comparison through adjustable transparency.

### 7.5 Rectangular Comparison Plots

For one-dimensional plots, the detail page now uses overlay comparison rather than side-by-side duplication. The comparison section includes:

1. speed profile overlay,
2. longitudinal-acceleration overlay, and
3. curvature overlay.

This keeps the spatial comparison square and side-by-side while reserving rectangular plots for direct curve overlays along arc length.

## 8. Validation Summary

The following focused validation steps were completed during implementation:

1. static diagnostics on all touched frontend files,
2. one-track preload and cache validation for all four methods,
3. persisted cache reload validation,
4. Ubuntu/Linux multiprocessing validation for method B,
5. overview ranking-page rendering checks,
6. track-detail side-by-side comparison rendering checks,
7. overlay rendering checks,
8. compare-mode toggle checks,
9. side-by-side left/right layout checks at desktop browser widths, and
10. synchronized zoom/pan checks on the side-by-side maps.

Representative method validation on a one-track preload produced the following lap times on BrandsHatch:

1. baseline: `114.192 s`
2. method A: `100.319 s`
3. method B: `99.992 s`
4. method C: `100.318 s`

This is consistent with the intended interpretation of the methods:

1. A materially improves the baseline,
2. B slightly improves on A through direct lap-time refinement, and
3. C closely matches A as a solver alternative.

## 9. User-Facing Outcome

From the browser user’s perspective, the dashboard now supports three distinct workflows instead of one:

1. inspect a single method for one track,
2. compare two methods on one track through side-by-side or overlay visualizations, and
3. compare whole-track rankings across methods from the overview page.

Because the heavy computations are now cached and reused, these workflows remain responsive after the initial preload.

## 10. Remaining Limitations

The current comparison layer operates on final cached method results, not full optimizer histories. As a result, the dashboard can compare final solutions across methods, but it cannot yet replay every optimization iteration for method A, B, or C.

If iteration-by-iteration playback is desired later, the cache schema will need to be extended so optimization histories are serialized along with the final payloads.

## 11. Concluding Remarks

This update moves the project from a baseline-only browser inspection tool to a cached, multimethod comparison dashboard. The main technical change is not only that more methods are available, but that the UI is now organized around comparison as a first-class workflow rather than as a side effect of switching selectors.

The backend remains numerically grounded in the same core analysis pipeline, while the frontend now supports faster visual interpretation of how trajectory optimization changes path geometry, lap time, and ranking behavior across the available track set.