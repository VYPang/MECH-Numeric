(function () {
  "use strict";

  const PLOT_LAYOUT_DEFAULTS = {
    paper_bgcolor: "#181b22",
    plot_bgcolor: "#181b22",
    font: { color: "#e6e8ee", family: "Inter, system-ui, sans-serif" },
    margin: { l: 60, r: 30, t: 30, b: 50 },
  };

  const TURBO_SPEED_COLORSCALE = [
    [0.0, "#2c3eec"],
    [0.2, "#1fa4ff"],
    [0.5, "#43d17a"],
    [0.8, "#f9b338"],
    [1.0, "#d7191c"],
  ];

  const METHOD_LABELS = {
    centerline_baseline: "Centerline baseline",
    min_curvature: "Min curvature (A)",
    min_lap_time: "Min lap time (B)",
    min_curvature_custom: "Min curvature custom (C)",
  };

  const DEFAULT_METHOD = "centerline_baseline";
  const DEFAULT_COMPARE_METHOD = "min_lap_time";
  const METHOD_COLORS = {
    centerline_baseline: "#f5f7fa",
    min_curvature: "#4cc2ff",
    min_lap_time: "#ffb86c",
    min_curvature_custom: "#58d68d",
  };
  let vehicleConfigPromise = null;

  function getJSON(url) {
    return fetch(url).then((response) => {
      if (!response.ok) {
        throw new Error(`Request failed (${response.status}): ${url}`);
      }
      return response.json();
    });
  }

  function getNested(obj, dotted) {
    return dotted.split(".").reduce((acc, key) => (acc == null ? acc : acc[key]), obj);
  }

  function formatNumber(value, decimals = 3) {
    if (value === null || value === undefined || Number.isNaN(value)) return "—";
    if (typeof value !== "number") return String(value);
    if (Number.isInteger(value) && decimals === 0) return value.toString();
    if (Math.abs(value) >= 1000) return value.toFixed(1);
    if (Math.abs(value) >= 10) return value.toFixed(2);
    return value.toFixed(decimals);
  }

  function formatSigned(value, decimals = 3) {
    if (value === null || value === undefined || Number.isNaN(value)) return "—";
    const sign = value > 0 ? "+" : "";
    return `${sign}${formatNumber(value, decimals)}`;
  }

  function percentDelta(compareValue, referenceValue) {
    if (!Number.isFinite(compareValue) || !Number.isFinite(referenceValue) || referenceValue === 0) return null;
    return ((compareValue - referenceValue) / referenceValue) * 100;
  }

  function methodColor(method) {
    return METHOD_COLORS[normalizeMethod(method)] || "#4cc2ff";
  }

  function getVehicleConfig() {
    if (!vehicleConfigPromise) {
      vehicleConfigPromise = getJSON("/api/vehicle").catch(() => null);
    }
    return vehicleConfigPromise;
  }

  function normalizeMethod(method) {
    return METHOD_LABELS[method] ? method : DEFAULT_METHOD;
  }

  function formatMethodLabel(method) {
    return METHOD_LABELS[normalizeMethod(method)];
  }

  function populateMethodSelect(selectEl, selectedMethod) {
    if (!selectEl) return;
    const method = normalizeMethod(selectedMethod);
    selectEl.innerHTML = Object.entries(METHOD_LABELS)
      .map(([value, label]) => `<option value="${value}">${label}</option>`)
      .join("");
    selectEl.value = method;
    selectEl.disabled = false;
  }

  function populateComparisonSelects(referenceSelect, compareSelect, referenceMethod, compareMethod) {
    populateMethodSelect(referenceSelect, referenceMethod);
    populateMethodSelect(compareSelect, compareMethod);
  }

  function wrapClosed(values) {
    if (!values.length) return [];
    return values.concat(values[0]);
  }

  function midpointCoordinates(xs, ys) {
    const points = [];
    for (let i = 0; i < xs.length; i += 1) {
      const next = (i + 1) % xs.length;
      points.push({ x: 0.5 * (xs[i] + xs[next]), y: 0.5 * (ys[i] + ys[next]) });
    }
    return points;
  }

  function computeTrackBoundaries(track) {
    const left = [];
    const right = [];

    for (let i = 0; i < track.x_m.length; i += 1) {
      const prev = (i - 1 + track.x_m.length) % track.x_m.length;
      const next = (i + 1) % track.x_m.length;
      const tangentX = track.x_m[next] - track.x_m[prev];
      const tangentY = track.y_m[next] - track.y_m[prev];
      const tangentNorm = Math.hypot(tangentX, tangentY) || 1.0;
      const normalX = -tangentY / tangentNorm;
      const normalY = tangentX / tangentNorm;

      left.push({
        x: track.x_m[i] + normalX * track.width_left_m[i],
        y: track.y_m[i] + normalY * track.width_left_m[i],
      });
      right.push({
        x: track.x_m[i] - normalX * track.width_right_m[i],
        y: track.y_m[i] - normalY * track.width_right_m[i],
      });
    }

    return { left, right };
  }

  function phaseState(value, threshold) {
    if (value > threshold) return "accel";
    if (value < -threshold) return "brake";
    return "neutral";
  }

  function extractTrackEvents(payload, threshold = 0.5) {
    const points = midpointCoordinates(payload.profile.x_m, payload.profile.y_m);
    const accel = [];
    const brake = [];
    let previous = "neutral";

    for (let i = 0; i < payload.speed.longitudinal_accel_mps2.length; i += 1) {
      const state = phaseState(payload.speed.longitudinal_accel_mps2[i], threshold);
      if (state !== previous) {
        if (state === "accel") {
          accel.push({ ...points[i], s: payload.speed.s_midpoints_m[i], value: payload.speed.longitudinal_accel_mps2[i] });
        }
        if (state === "brake") {
          brake.push({ ...points[i], s: payload.speed.s_midpoints_m[i], value: payload.speed.longitudinal_accel_mps2[i] });
        }
      }
      previous = state;
    }

    return { accel, brake };
  }

  function getEventCounts(payload) {
    const events = extractTrackEvents(payload);
    return {
      accel: events.accel.length,
      brake: events.brake.length,
      transitions: payload.metrics.performance.phase_transition_count,
    };
  }

  function computeStartArrow(payload) {
    const xs = payload.profile.x_m;
    const ys = payload.profile.y_m;
    if (xs.length < 2) {
      return { tailX: xs[0] ?? 0, tailY: ys[0] ?? 0, headX: xs[0] ?? 0, headY: ys[0] ?? 0 };
    }

    const lookahead = Math.min(Math.max(8, Math.floor(xs.length * 0.03)), xs.length - 1);
    const dx = xs[lookahead] - xs[0];
    const dy = ys[lookahead] - ys[0];
    const norm = Math.hypot(dx, dy) || 1.0;

    const xSpan = Math.max(...xs) - Math.min(...xs);
    const ySpan = Math.max(...ys) - Math.min(...ys);
    const arrowLength = 0.08 * Math.max(xSpan, ySpan);

    return {
      tailX: xs[0],
      tailY: ys[0],
      headX: xs[0] + (dx / norm) * arrowLength,
      headY: ys[0] + (dy / norm) * arrowLength,
    };
  }

  function clampOpacityPercent(value, fallback) {
    if (value === null || value === undefined || value === "") return fallback;
    const numeric = Number(value);
    if (!Number.isFinite(numeric)) return fallback;
    return Math.max(10, Math.min(100, Math.round(numeric)));
  }

  function sharedSpeedRange(vehicleConfig, ...payloads) {
    if (vehicleConfig && Number.isFinite(vehicleConfig.v_min_mps) && Number.isFinite(vehicleConfig.v_max_mps)) {
      return {
        cmin: vehicleConfig.v_min_mps,
        cmax: vehicleConfig.v_max_mps > vehicleConfig.v_min_mps ? vehicleConfig.v_max_mps : vehicleConfig.v_min_mps + 1,
      };
    }

    const values = payloads
      .flatMap((payload) => {
        if (!payload || !payload.speed) return [];
        return [
          ...(payload.speed.final_speed_mps || []),
          ...(payload.speed.speed_cap_mps || []),
          ...(payload.speed.forward_speed_mps || []),
        ];
      })
      .filter((value) => Number.isFinite(value));
    const min = values.length ? Math.min(...values) : 0;
    const max = values.length ? Math.max(...values) : 1;
    return {
      cmin: min,
      cmax: max > min ? max : min + 1,
    };
  }

  function buildTrackMapFigure(payload, speedRange, options = {}) {
    const showLegend = options.showLegend ?? true;
    const titleText = options.title ?? null;
    const showColorBar = options.showColorBar ?? true;
    const colorBarTitle = options.colorBarTitle ?? "Speed (m/s)";
    const margin = options.margin ?? { l: 40, r: 28, t: titleText ? 42 : 28, b: 40 };

    const track = payload.track;
    const speed = payload.speed;
    const nodeSpeed = speed.final_speed_mps.slice(0, payload.profile.x_m.length);
    const boundaries = computeTrackBoundaries(track);
    const events = extractTrackEvents(payload);
    const startArrow = computeStartArrow(payload);
    const traces = [
      {
        x: wrapClosed(boundaries.left.map((point) => point.x)),
        y: wrapClosed(boundaries.left.map((point) => point.y)),
        mode: "lines",
        line: { color: "#667085", width: 1.2 },
        name: "Left boundary",
        hoverinfo: "skip",
      },
      {
        x: wrapClosed(boundaries.right.map((point) => point.x)),
        y: wrapClosed(boundaries.right.map((point) => point.y)),
        mode: "lines",
        line: { color: "#667085", width: 1.2 },
        name: "Right boundary",
        hoverinfo: "skip",
      },
      {
        x: wrapClosed(payload.profile.x_m),
        y: wrapClosed(payload.profile.y_m),
        mode: "lines",
        line: { color: "rgba(255,255,255,0.28)", width: 0.9 },
        name: "Trajectory",
        hoverinfo: "skip",
      },
      {
        x: payload.profile.x_m,
        y: payload.profile.y_m,
        mode: "markers",
        marker: {
          size: 5,
          color: nodeSpeed,
          colorscale: TURBO_SPEED_COLORSCALE,
          cmin: speedRange.cmin,
          cmax: speedRange.cmax,
          showscale: showColorBar,
          colorbar: showColorBar ? { title: colorBarTitle, thickness: 14 } : undefined,
        },
        text: nodeSpeed.map(
          (value, index) => `s=${formatNumber(payload.speed.s_nodes_m[index])} m<br>v=${formatNumber(value)} m/s`,
        ),
        hoverinfo: "text",
        name: "Speed sample",
      },
      {
        x: events.accel.map((point) => point.x),
        y: events.accel.map((point) => point.y),
        mode: "markers+text",
        marker: { size: 10, color: "#58d68d", line: { color: "#0f1115", width: 1 } },
        text: events.accel.map(() => "A"),
        textposition: "top center",
        textfont: { color: "#58d68d", size: 11 },
        hovertext: events.accel.map(
          (point) => `Accel point<br>s=${formatNumber(point.s)} m<br>a=${formatNumber(point.value)} m/s²`,
        ),
        hoverinfo: "text",
        name: "Accel points",
      },
      {
        x: events.brake.map((point) => point.x),
        y: events.brake.map((point) => point.y),
        mode: "markers+text",
        marker: { size: 10, color: "#ff6b6b", symbol: "diamond", line: { color: "#0f1115", width: 1 } },
        text: events.brake.map(() => "B"),
        textposition: "bottom center",
        textfont: { color: "#ff6b6b", size: 11 },
        hovertext: events.brake.map(
          (point) => `Brake point<br>s=${formatNumber(point.s)} m<br>a=${formatNumber(point.value)} m/s²`,
        ),
        hoverinfo: "text",
        name: "Brake points",
      },
      {
        x: [payload.profile.x_m[0]],
        y: [payload.profile.y_m[0]],
        mode: "markers",
        marker: { size: 10, color: "#ff4040", symbol: "x" },
        name: "Start (highlight)",
        hoverinfo: "skip",
        showlegend: false,
      },
    ];

    return {
      traces,
      layout: Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        title: titleText ? { text: titleText, font: { size: 13 } } : undefined,
        margin,
        yaxis: { scaleanchor: "x", scaleratio: 1, title: "y (m)" },
        xaxis: { title: "x (m)" },
        showlegend: showLegend,
        legend: showLegend ? { orientation: "h", y: 1.08, x: 0.02 } : undefined,
        annotations: [
          {
            x: startArrow.tailX,
            y: startArrow.tailY,
            xref: "x",
            yref: "y",
            text: "Start",
            showarrow: false,
            xanchor: "left",
            yanchor: "bottom",
            xshift: 6,
            yshift: 6,
            font: { color: "#f5f7fa", size: 12 },
          },
          {
            x: startArrow.headX,
            y: startArrow.headY,
            ax: startArrow.tailX,
            ay: startArrow.tailY,
            xref: "x",
            yref: "y",
            axref: "x",
            ayref: "y",
            arrowhead: 3,
            arrowsize: 1.3,
            arrowwidth: 2.2,
            arrowcolor: "#f5f7fa",
            text: "",
          },
        ],
      }),
    };
  }

  // ---------- Overview ----------

  function renderOverview() {
    const params = new URLSearchParams(window.location.search);
    const methodSelect = document.getElementById("method-select");
    const tbody = document.querySelector("#rankings-table tbody");
    getJSON("/api/vehicle")
      .then((vehicle) => {
        const el = document.getElementById("vehicle-summary");
        if (!el) return;
        el.textContent =
          `ay_max=${vehicle.ay_max_mps2} m/s² | ax_eng=${vehicle.ax_engine_max_mps2} m/s²` +
          ` | brake=${vehicle.brake_max_mps2} m/s² | μ=${vehicle.mu}` +
          ` | P=${vehicle.power_limit_w ? (vehicle.power_limit_w / 1000).toFixed(0) + " kW" : "n/a"}` +
          ` | v_max=${vehicle.v_max_mps} m/s`;
      })
      .catch(() => {});

    const status = document.getElementById("rankings-status");
    let activeMethod = normalizeMethod(params.get("method"));
    let rows = [];
    let sortKey = "difficulty_score";
    let sortDir = -1;

    populateMethodSelect(methodSelect, activeMethod);

    function rerender() {
      rows.sort((a, b) => {
        const aValue = getNested(a, sortKey);
        const bValue = getNested(b, sortKey);
        if (typeof aValue === "string") {
          return sortDir * aValue.localeCompare(bValue);
        }
        return sortDir * ((aValue ?? 0) - (bValue ?? 0));
      });
      tbody.innerHTML = rows
        .map((row) => {
          const href = `/track?name=${encodeURIComponent(row.track_name)}&method=${encodeURIComponent(activeMethod)}`;
          return (
            `<tr data-track="${row.track_name}">` +
            `<td><a href="${href}">${row.track_name}</a></td>` +
            `<td>${formatNumber(row.difficulty_score)}</td>` +
            `<td>${formatNumber(row.performance.lap_time_s)}</td>` +
            `<td>${formatNumber(row.performance.average_speed_mps)}</td>` +
            `<td>${formatNumber(row.performance.max_speed_mps)}</td>` +
            `<td>${formatNumber(row.performance.min_speed_mps)}</td>` +
            `<td>${formatNumber(row.performance.speed_stdev_mps)}</td>` +
            `<td>${formatNumber(row.performance.mean_abs_long_accel_mps2)}</td>` +
            `<td>${formatNumber(row.performance.lateral_limit_fraction * 100)}</td>` +
            `<td>${formatNumber(row.geometry.total_length_m)}</td>` +
            `<td>${row.geometry.rms_curvature.toExponential(2)}</td>` +
            `<td>${formatNumber(row.geometry.corner_severity_index, 5)}</td>` +
            `</tr>`
          );
        })
        .join("");

      tbody.querySelectorAll("tr").forEach((tr) => {
        tr.addEventListener("click", (event) => {
          if (event.target.tagName === "A") return;
          window.location.href = `/track?name=${encodeURIComponent(tr.dataset.track)}&method=${encodeURIComponent(activeMethod)}`;
        });
      });
    }

    function loadRankings() {
      status.textContent = `Loading ${formatMethodLabel(activeMethod)} rankings…`;
      tbody.innerHTML = "";
      getJSON(`/api/rankings?method=${encodeURIComponent(activeMethod)}`)
        .then((payload) => {
          activeMethod = normalizeMethod(payload.method);
          if (methodSelect) methodSelect.value = activeMethod;
          rows = payload.rows.slice();
          rerender();
          status.textContent = `Loaded ${rows.length} tracks for ${formatMethodLabel(activeMethod)}.`;
        })
        .catch((error) => {
          status.textContent = `Failed to load rankings: ${error.message}`;
        });
    }

    if (methodSelect) {
      methodSelect.addEventListener("change", () => {
        activeMethod = normalizeMethod(methodSelect.value);
        params.set("method", activeMethod);
        window.history.replaceState({}, "", `${window.location.pathname}?${params.toString()}`);
        loadRankings();
      });
    }

    document.querySelectorAll("#rankings-table th").forEach((th) => {
      if (th.dataset.bound === "1") return;
      th.dataset.bound = "1";
      th.addEventListener("click", () => {
        const key = th.dataset.key;
        if (!key) return;
        if (sortKey === key) {
          sortDir *= -1;
        } else {
          sortKey = key;
          sortDir = key === "track_name" ? 1 : -1;
        }
        rerender();
      });
    });

    loadRankings();
    renderOverviewComparison(params);
  }

  function renderOverviewComparison(params) {
    const referenceSelect = document.getElementById("comparison-reference-select");
    const compareSelect = document.getElementById("comparison-compare-select");
    const tbody = document.querySelector("#comparison-table tbody");
    const status = document.getElementById("comparison-status");
    if (!referenceSelect || !compareSelect || !tbody || !status) return;

    let referenceMethod = normalizeMethod(params.get("reference_method") || DEFAULT_METHOD);
    let compareMethod = normalizeMethod(params.get("compare_method") || DEFAULT_COMPARE_METHOD);
    let rows = [];
    let sortKey = "rank_shift";
    let sortDir = -1;

    populateComparisonSelects(referenceSelect, compareSelect, referenceMethod, compareMethod);

    function updateUrl() {
      params.set("reference_method", referenceMethod);
      params.set("compare_method", compareMethod);
      window.history.replaceState({}, "", `${window.location.pathname}?${params.toString()}`);
    }

    function rerender() {
      rows.sort((a, b) => {
        const aValue = getNested(a, sortKey);
        const bValue = getNested(b, sortKey);
        if (typeof aValue === "string") return sortDir * aValue.localeCompare(bValue);
        return sortDir * ((aValue ?? 0) - (bValue ?? 0));
      });

      tbody.innerHTML = rows
        .map((row) => {
          const href = `/track?name=${encodeURIComponent(row.track_name)}` +
            `&method=${encodeURIComponent(compareMethod)}` +
            `&reference_method=${encodeURIComponent(referenceMethod)}` +
            `&compare_method=${encodeURIComponent(compareMethod)}` +
            `&comparison_mode=1&comparison_view=side_by_side`;
          return (
            `<tr data-track="${row.track_name}">` +
            `<td><a href="${href}">${row.track_name}</a></td>` +
            `<td>${row.reference_rank}</td>` +
            `<td>${row.compare_rank}</td>` +
            `<td>${formatSigned(row.rank_shift, 0)}</td>` +
            `<td>${formatSigned(row.difficulty_delta)}</td>` +
            `<td>${formatSigned(row.lap_time_delta_s)}</td>` +
            `<td>${formatSigned(row.lap_time_delta_pct, 2)}</td>` +
            `<td>${formatSigned(row.transition_delta, 0)}</td>` +
            `<td>${row.rms_curvature_delta.toExponential(2)}</td>` +
            `<td>${formatSigned(row.length_delta_m)}</td>` +
            `</tr>`
          );
        })
        .join("");

      tbody.querySelectorAll("tr").forEach((tr) => {
        tr.addEventListener("click", (event) => {
          if (event.target.tagName === "A") return;
          window.location.href = `/track?name=${encodeURIComponent(tr.dataset.track)}` +
            `&method=${encodeURIComponent(compareMethod)}` +
            `&reference_method=${encodeURIComponent(referenceMethod)}` +
            `&compare_method=${encodeURIComponent(compareMethod)}` +
            `&comparison_mode=1&comparison_view=side_by_side`;
        });
      });
      renderDifficultyComparisonPlot(rows, referenceMethod, compareMethod);
    }

    function buildRows(referencePayload, comparePayload) {
      const compareByTrack = new Map(comparePayload.rows.map((row, index) => [row.track_name, { row, rank: index + 1 }]));
      return referencePayload.rows
        .map((referenceRow, index) => {
          const compareEntry = compareByTrack.get(referenceRow.track_name);
          if (!compareEntry) return null;
          const compareRow = compareEntry.row;
          const referenceRank = index + 1;
          const compareRank = compareEntry.rank;
          const referenceLap = referenceRow.performance.lap_time_s;
          const compareLap = compareRow.performance.lap_time_s;
          return {
            track_name: referenceRow.track_name,
            reference_rank: referenceRank,
            compare_rank: compareRank,
            rank_shift: referenceRank - compareRank,
            reference_difficulty: referenceRow.difficulty_score,
            compare_difficulty: compareRow.difficulty_score,
            difficulty_delta: compareRow.difficulty_score - referenceRow.difficulty_score,
            lap_time_delta_s: compareLap - referenceLap,
            lap_time_delta_pct: percentDelta(compareLap, referenceLap),
            transition_delta: compareRow.performance.phase_transition_count - referenceRow.performance.phase_transition_count,
            rms_curvature_delta: compareRow.geometry.rms_curvature - referenceRow.geometry.rms_curvature,
            length_delta_m: compareRow.geometry.total_length_m - referenceRow.geometry.total_length_m,
          };
        })
        .filter(Boolean);
    }

    function loadComparison() {
      updateUrl();
      status.textContent = `Comparing ${formatMethodLabel(compareMethod)} against ${formatMethodLabel(referenceMethod)}…`;
      tbody.innerHTML = "";
      Promise.all([
        getJSON(`/api/rankings?method=${encodeURIComponent(referenceMethod)}`),
        getJSON(`/api/rankings?method=${encodeURIComponent(compareMethod)}`),
      ])
        .then(([referencePayload, comparePayload]) => {
          rows = buildRows(referencePayload, comparePayload);
          rerender();
          status.textContent = `Compared ${rows.length} tracks. Positive rank shift means the compared method moved the track toward harder ranks.`;
        })
        .catch((error) => {
          status.textContent = `Failed to load method comparison: ${error.message}`;
        });
    }

    [referenceSelect, compareSelect].forEach((selectEl) => {
      selectEl.addEventListener("change", () => {
        referenceMethod = normalizeMethod(referenceSelect.value);
        compareMethod = normalizeMethod(compareSelect.value);
        loadComparison();
      });
    });

    document.querySelectorAll("#comparison-table th").forEach((th) => {
      if (th.dataset.bound === "1") return;
      th.dataset.bound = "1";
      th.addEventListener("click", () => {
        const key = th.dataset.key;
        if (!key) return;
        if (sortKey === key) {
          sortDir *= -1;
        } else {
          sortKey = key;
          sortDir = key === "track_name" ? 1 : -1;
        }
        rerender();
      });
    });

    loadComparison();
  }

  function renderDifficultyComparisonPlot(rows, referenceMethod, compareMethod) {
    const div = document.getElementById("plot-difficulty-comparison");
    if (!div || !window.Plotly) return;
    const values = rows.flatMap((row) => [row.reference_difficulty, row.compare_difficulty]);
    const minValue = Math.min(...values, 0);
    const maxValue = Math.max(...values, 1);
    const padding = (maxValue - minValue) * 0.08 || 0.05;
    const axisMin = minValue - padding;
    const axisMax = maxValue + padding;
    const traces = [
      {
        x: rows.map((row) => row.reference_difficulty),
        y: rows.map((row) => row.compare_difficulty),
        mode: "markers",
        marker: {
          size: 9,
          color: rows.map((row) => row.rank_shift),
          colorscale: [
            [0, "#58d68d"],
            [0.5, "#e6e8ee"],
            [1, "#ffb86c"],
          ],
          colorbar: { title: "Rank shift" },
        },
        text: rows.map(
          (row) => `${row.track_name}<br>rank shift=${formatSigned(row.rank_shift, 0)}<br>lap Δ=${formatSigned(row.lap_time_delta_s)} s`,
        ),
        hoverinfo: "text",
        name: "",
        showlegend: false,
      },
      {
        x: [axisMin, axisMax],
        y: [axisMin, axisMax],
        mode: "lines",
        line: { color: "#667085", dash: "dash" },
        hoverinfo: "skip",
        name: "",
        showlegend: false,
      },
    ];
    Plotly.newPlot(
      div,
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: `${formatMethodLabel(referenceMethod)} difficulty`, range: [axisMin, axisMax] },
        yaxis: { title: `${formatMethodLabel(compareMethod)} difficulty`, range: [axisMin, axisMax] },
        hovermode: "closest",
        showlegend: false,
      }),
      { responsive: true },
    );
  }

  // ---------- Track detail ----------

  function renderTrackDetail() {
    const params = new URLSearchParams(window.location.search);
    const trackName = params.get("name");
    const methodSelect = document.getElementById("track-method-select");
    const comparisonModeToggle = document.getElementById("track-comparison-mode-toggle");
    const referenceSelect = document.getElementById("track-reference-select");
    const compareSelect = document.getElementById("track-compare-select");
    const comparisonViewSelect = document.getElementById("track-comparison-view-select");
    const overlayControls = document.getElementById("comparison-overlay-controls");
    const referenceOpacitySlider = document.getElementById("reference-opacity-slider");
    const compareOpacitySlider = document.getElementById("compare-opacity-slider");
    const referenceOpacityLabel = document.getElementById("reference-opacity-label");
    const compareOpacityLabel = document.getElementById("compare-opacity-label");
    const referenceOpacityValue = document.getElementById("reference-opacity-value");
    const compareOpacityValue = document.getElementById("compare-opacity-value");
    const singleMethodControls = document.getElementById("single-method-controls");
    const singleTrackContent = document.getElementById("single-track-content");
    const comparisonContent = document.getElementById("comparison-content");
    const comparisonControlLabels = [
      referenceSelect?.closest("label"),
      compareSelect?.closest("label"),
      comparisonViewSelect?.closest("label"),
    ].filter(Boolean);
    const backLink = document.getElementById("track-back-link");
    if (!trackName) {
      document.getElementById("track-title").textContent = "Missing track name";
      return;
    }

    let activeMethod = normalizeMethod(params.get("method"));
    let referenceMethod = normalizeMethod(params.get("reference_method") || DEFAULT_METHOD);
    let compareMethod = normalizeMethod(params.get("compare_method") || activeMethod || DEFAULT_COMPARE_METHOD);
    if (compareMethod === DEFAULT_METHOD && activeMethod === DEFAULT_METHOD) compareMethod = DEFAULT_COMPARE_METHOD;
    let comparisonView = params.get("comparison_view") === "overlay" ? "overlay" : "side_by_side";
    let comparisonMode = params.get("comparison_mode") === "1";
    let referenceOpacity = clampOpacityPercent(params.get("reference_opacity"), 55);
    let compareOpacity = clampOpacityPercent(params.get("compare_opacity"), 85);
    const vehiclePromise = getVehicleConfig();

    document.getElementById("track-title").textContent = trackName;
    populateMethodSelect(methodSelect, activeMethod);
    populateComparisonSelects(referenceSelect, compareSelect, referenceMethod, compareMethod);
    if (comparisonViewSelect) comparisonViewSelect.value = comparisonView;
    if (comparisonModeToggle) comparisonModeToggle.checked = comparisonMode;
    if (referenceOpacitySlider) referenceOpacitySlider.value = String(referenceOpacity);
    if (compareOpacitySlider) compareOpacitySlider.value = String(compareOpacity);

    function syncOverlayControlText() {
      if (referenceOpacityLabel) referenceOpacityLabel.textContent = `${formatMethodLabel(referenceMethod)} opacity`;
      if (compareOpacityLabel) compareOpacityLabel.textContent = `${formatMethodLabel(compareMethod)} opacity`;
      if (referenceOpacityValue) referenceOpacityValue.textContent = `${referenceOpacity}%`;
      if (compareOpacityValue) compareOpacityValue.textContent = `${compareOpacity}%`;
    }

    function applyTrackDisplayMode() {
      if (singleMethodControls) singleMethodControls.hidden = comparisonMode;
      if (singleTrackContent) singleTrackContent.hidden = comparisonMode;
      if (comparisonContent) {
        comparisonContent.hidden = !comparisonMode;
        comparisonContent.classList.toggle("comparison-mode-active", comparisonMode);
      }
      comparisonControlLabels.forEach((label) => {
        label.hidden = !comparisonMode;
      });
      if (overlayControls) {
        overlayControls.hidden = !comparisonMode || comparisonView !== "overlay";
      }
      syncOverlayControlText();
    }

    function updateTrackUrl() {
      params.set("name", trackName);
      params.set("method", activeMethod);
      params.set("reference_method", referenceMethod);
      params.set("compare_method", compareMethod);
      params.set("comparison_view", comparisonView);
      if (comparisonMode) params.set("comparison_mode", "1");
      else params.delete("comparison_mode");
      params.set("reference_opacity", String(referenceOpacity));
      params.set("compare_opacity", String(compareOpacity));
      window.history.replaceState({}, "", `${window.location.pathname}?${params.toString()}`);
      if (backLink) {
        backLink.href = `/?method=${encodeURIComponent(activeMethod)}` +
          `&reference_method=${encodeURIComponent(referenceMethod)}` +
          `&compare_method=${encodeURIComponent(compareMethod)}`;
      }
    }

    function loadTrackDetail() {
      document.getElementById("track-subtitle").textContent =
        `Loading ${formatMethodLabel(activeMethod)} solution…`;
      Promise.all([
        getJSON(`/api/tracks/${encodeURIComponent(trackName)}/analysis?method=${encodeURIComponent(activeMethod)}`),
        vehiclePromise,
      ])
        .then(([payload, vehicleConfig]) => {
          activeMethod = normalizeMethod(payload.method);
          if (methodSelect) methodSelect.value = activeMethod;
          updateTrackUrl();
          document.getElementById("track-subtitle").textContent =
            `Geometry: ${payload.geometry_method}`;
          renderMetricCards(payload);
          renderTrackMap(payload, sharedSpeedRange(vehicleConfig, payload));
          renderSpeedPlot(payload);
          renderLongAccelPlot(payload);
          renderCurvaturePlot(payload);
          renderNumerics(payload);
          attachLinkedHover(payload);
        })
        .catch((error) => {
          document.getElementById("track-subtitle").textContent =
            `Failed to load: ${error.message}`;
        });
    }

    if (methodSelect) {
      methodSelect.addEventListener("change", () => {
        activeMethod = normalizeMethod(methodSelect.value);
        if (comparisonMode) {
          compareMethod = activeMethod;
          if (compareSelect) compareSelect.value = compareMethod;
        }
        updateTrackUrl();
        loadTrackDetail();
        if (comparisonMode) loadTrackComparison();
      });
    }

    [referenceSelect, compareSelect].forEach((selectEl) => {
      if (!selectEl) return;
      selectEl.addEventListener("change", () => {
        referenceMethod = normalizeMethod(referenceSelect.value);
        compareMethod = normalizeMethod(compareSelect.value);
        syncOverlayControlText();
        updateTrackUrl();
        if (comparisonMode) loadTrackComparison();
      });
    });

    if (comparisonViewSelect) {
      comparisonViewSelect.addEventListener("change", () => {
        comparisonView = comparisonViewSelect.value === "side_by_side" ? "side_by_side" : "overlay";
        applyTrackDisplayMode();
        updateTrackUrl();
        if (comparisonMode) loadTrackComparison();
      });
    }

    if (comparisonModeToggle) {
      comparisonModeToggle.addEventListener("change", () => {
        comparisonMode = comparisonModeToggle.checked;
        applyTrackDisplayMode();
        updateTrackUrl();
        if (comparisonMode) loadTrackComparison();
      });
    }

    if (referenceOpacitySlider) {
      referenceOpacitySlider.addEventListener("input", () => {
        referenceOpacity = clampOpacityPercent(referenceOpacitySlider.value, referenceOpacity);
        syncOverlayControlText();
        updateTrackUrl();
        if (comparisonMode && comparisonView === "overlay") loadTrackComparison();
      });
    }

    if (compareOpacitySlider) {
      compareOpacitySlider.addEventListener("input", () => {
        compareOpacity = clampOpacityPercent(compareOpacitySlider.value, compareOpacity);
        syncOverlayControlText();
        updateTrackUrl();
        if (comparisonMode && comparisonView === "overlay") loadTrackComparison();
      });
    }

    function loadTrackComparison() {
      if (!comparisonMode) return;
      const status = document.getElementById("track-comparison-status");
      if (status) {
        status.textContent = `Comparing ${formatMethodLabel(compareMethod)} against ${formatMethodLabel(referenceMethod)}…`;
      }
      Promise.all([
        getJSON(`/api/tracks/${encodeURIComponent(trackName)}/analysis?method=${encodeURIComponent(referenceMethod)}`),
        getJSON(`/api/tracks/${encodeURIComponent(trackName)}/analysis?method=${encodeURIComponent(compareMethod)}`),
        vehiclePromise,
      ])
        .then(([referencePayload, comparePayload, vehicleConfig]) => {
          renderComparisonMetricCards(referencePayload, comparePayload);
          renderTrackComparison(
            referencePayload,
            comparePayload,
            comparisonView,
            vehicleConfig,
            {
              referenceOpacity: referenceOpacity / 100,
              compareOpacity: compareOpacity / 100,
            },
          );
          renderSpeedComparison(referencePayload, comparePayload);
          renderLongAccelComparison(referencePayload, comparePayload);
          renderCurvatureComparison(referencePayload, comparePayload);
          if (status) {
            status.textContent = `Compared ${formatMethodLabel(compareMethod)} against ${formatMethodLabel(referenceMethod)}.`;
          }
        })
        .catch((error) => {
          if (status) status.textContent = `Failed to load comparison: ${error.message}`;
        });
    }

    applyTrackDisplayMode();
    updateTrackUrl();
    loadTrackDetail();
    if (comparisonMode) loadTrackComparison();
  }

  function renderComparisonMetricCards(referencePayload, comparePayload) {
    const container = document.getElementById("comparison-metric-cards");
    if (!container) return;
    const ref = referencePayload.metrics;
    const cmp = comparePayload.metrics;
    const referenceEvents = getEventCounts(referencePayload);
    const compareEvents = getEventCounts(comparePayload);
    const lapDelta = cmp.performance.lap_time_s - ref.performance.lap_time_s;
    const lapDeltaPct = percentDelta(cmp.performance.lap_time_s, ref.performance.lap_time_s);
    const cards = [
      ["Lap time Δ", `${formatSigned(lapDelta)} s`, lapDelta],
      ["Lap time Δ%", `${formatSigned(lapDeltaPct, 2)}%`, lapDeltaPct],
      ["Lap length Δ", `${formatSigned(cmp.geometry.total_length_m - ref.geometry.total_length_m)} m`, cmp.geometry.total_length_m - ref.geometry.total_length_m],
      ["κ rms Δ", (cmp.geometry.rms_curvature - ref.geometry.rms_curvature).toExponential(2), cmp.geometry.rms_curvature - ref.geometry.rms_curvature],
      ["Accel points Δ", formatSigned(compareEvents.accel - referenceEvents.accel, 0), compareEvents.accel - referenceEvents.accel],
      ["Brake points Δ", formatSigned(compareEvents.brake - referenceEvents.brake, 0), compareEvents.brake - referenceEvents.brake],
      ["Phase transitions Δ", formatSigned(compareEvents.transitions - referenceEvents.transitions, 0), compareEvents.transitions - referenceEvents.transitions],
      ["Mean |aₓ| Δ", `${formatSigned(cmp.performance.mean_abs_long_accel_mps2 - ref.performance.mean_abs_long_accel_mps2)} m/s²`, cmp.performance.mean_abs_long_accel_mps2 - ref.performance.mean_abs_long_accel_mps2],
    ];
    container.innerHTML = cards
      .map(([label, value, delta]) => {
        const className = delta > 0 ? "delta-positive" : delta < 0 ? "delta-negative" : "";
        return `<div class="metric-card ${className}"><div class="label">${label}</div><div class="value">${value}</div></div>`;
      })
      .join("");
  }

  function renderTrackComparison(referencePayload, comparePayload, view, vehicleConfig, overlayOpacity) {
    const overlayPanel = document.getElementById("comparison-overlay-panel");
    const sidePanel = document.getElementById("comparison-side-by-side-panel");
    if (!overlayPanel || !sidePanel) return;
    const sideBySide = view === "side_by_side";
    overlayPanel.hidden = sideBySide;
    sidePanel.hidden = !sideBySide;
    const speedRange = sharedSpeedRange(vehicleConfig, referencePayload, comparePayload);
    if (sideBySide) {
      renderSingleComparisonMap(
        "plot-track-comparison-reference",
        referencePayload,
        referencePayload.method,
        speedRange,
      );
      renderSingleComparisonMap(
        "plot-track-comparison-compare",
        comparePayload,
        comparePayload.method,
        speedRange,
      );
      const referenceMap = document.getElementById("plot-track-comparison-reference");
      const compareMap = document.getElementById("plot-track-comparison-compare");
      if (referenceMap) delete referenceMap.dataset.zoomSyncBound;
      if (compareMap) delete compareMap.dataset.zoomSyncBound;
      bindMapZoomSync("plot-track-comparison-reference", "plot-track-comparison-compare");
      return;
    }
    renderOverlayComparisonMap(referencePayload, comparePayload, speedRange, overlayOpacity);
  }

  function boundaryTraces(track) {
    const boundaries = computeTrackBoundaries(track);
    return [
      {
        x: wrapClosed(boundaries.left.map((point) => point.x)),
        y: wrapClosed(boundaries.left.map((point) => point.y)),
        mode: "lines",
        line: { color: "#667085", width: 1.1 },
        name: "Left boundary",
        hoverinfo: "skip",
      },
      {
        x: wrapClosed(boundaries.right.map((point) => point.x)),
        y: wrapClosed(boundaries.right.map((point) => point.y)),
        mode: "lines",
        line: { color: "#667085", width: 1.1 },
        name: "Right boundary",
        hoverinfo: "skip",
      },
    ];
  }

  function renderOverlayComparisonMap(referencePayload, comparePayload, speedRange, overlayOpacity) {
    const div = document.getElementById("plot-track-comparison-overlay");
    if (!div || !window.Plotly) return;
    const referenceSpeed = referencePayload.speed.final_speed_mps.slice(0, referencePayload.profile.x_m.length);
    const compareSpeed = comparePayload.speed.final_speed_mps.slice(0, comparePayload.profile.x_m.length);
    const referenceOpacity = overlayOpacity.referenceOpacity;
    const compareOpacity = overlayOpacity.compareOpacity;
    const referenceEvents = extractTrackEvents(referencePayload);
    const compareEvents = extractTrackEvents(comparePayload);
    const startArrow = computeStartArrow(referencePayload);

    function eventTrace(points, text, colorRgb, opacity, position, name, hoverPrefix) {
      return {
        x: points.map((point) => point.x),
        y: points.map((point) => point.y),
        mode: "markers+text",
        marker: {
          size: 10,
          color: `rgba(${colorRgb},${opacity})`,
          symbol: text === "B" ? "diamond" : "circle",
          line: { color: "#0f1115", width: 1 },
        },
        text: points.map(() => text),
        textposition: position,
        textfont: {
          color: `rgba(${colorRgb},${Math.max(opacity, 0.45)})`,
          size: 11,
        },
        hovertext: points.map(
          (point) => `${hoverPrefix}<br>s=${formatNumber(point.s)} m<br>a=${formatNumber(point.value)} m/s²`,
        ),
        hoverinfo: "text",
        name,
      };
    }

    const traces = [
      ...boundaryTraces(referencePayload.track),
      {
        x: wrapClosed(referencePayload.profile.x_m),
        y: wrapClosed(referencePayload.profile.y_m),
        mode: "lines",
        line: { color: `rgba(255,255,255,${Math.max(0.12, referenceOpacity * 0.65)})`, width: 1.0 },
        name: `${formatMethodLabel(referencePayload.method)} path`,
        hoverinfo: "skip",
      },
      {
        x: referencePayload.profile.x_m,
        y: referencePayload.profile.y_m,
        mode: "markers",
        marker: {
          size: 5,
          color: referenceSpeed,
          colorscale: TURBO_SPEED_COLORSCALE,
          cmin: speedRange.cmin,
          cmax: speedRange.cmax,
          showscale: true,
          colorbar: { title: "Speed (m/s)", thickness: 12, len: 0.72, x: 1.02 },
          opacity: referenceOpacity,
        },
        text: referenceSpeed.map(
          (value, index) => `${formatMethodLabel(referencePayload.method)}<br>s=${formatNumber(referencePayload.speed.s_nodes_m[index])} m<br>v=${formatNumber(value)} m/s`,
        ),
        hoverinfo: "text",
        name: `${formatMethodLabel(referencePayload.method)} speed`,
      },
      {
        x: wrapClosed(comparePayload.profile.x_m),
        y: wrapClosed(comparePayload.profile.y_m),
        mode: "lines",
        line: { color: `rgba(255,255,255,${Math.max(0.12, compareOpacity * 0.65)})`, width: 1.0 },
        name: `${formatMethodLabel(comparePayload.method)} path`,
        hoverinfo: "skip",
      },
      {
        x: comparePayload.profile.x_m,
        y: comparePayload.profile.y_m,
        mode: "markers",
        marker: {
          size: 5,
          color: compareSpeed,
          colorscale: TURBO_SPEED_COLORSCALE,
          cmin: speedRange.cmin,
          cmax: speedRange.cmax,
          showscale: false,
          opacity: compareOpacity,
        },
        text: compareSpeed.map(
          (value, index) => `${formatMethodLabel(comparePayload.method)}<br>s=${formatNumber(comparePayload.speed.s_nodes_m[index])} m<br>v=${formatNumber(value)} m/s`,
        ),
        hoverinfo: "text",
        name: `${formatMethodLabel(comparePayload.method)} speed`,
      },
      eventTrace(
        referenceEvents.accel,
        "A",
        "88,214,141",
        referenceOpacity,
        "top center",
        `${formatMethodLabel(referencePayload.method)} accel points`,
        `${formatMethodLabel(referencePayload.method)} accel point`,
      ),
      eventTrace(
        referenceEvents.brake,
        "B",
        "255,107,107",
        referenceOpacity,
        "bottom center",
        `${formatMethodLabel(referencePayload.method)} brake points`,
        `${formatMethodLabel(referencePayload.method)} brake point`,
      ),
      eventTrace(
        compareEvents.accel,
        "A",
        "88,214,141",
        compareOpacity,
        "top center",
        `${formatMethodLabel(comparePayload.method)} accel points`,
        `${formatMethodLabel(comparePayload.method)} accel point`,
      ),
      eventTrace(
        compareEvents.brake,
        "B",
        "255,107,107",
        compareOpacity,
        "bottom center",
        `${formatMethodLabel(comparePayload.method)} brake points`,
        `${formatMethodLabel(comparePayload.method)} brake point`,
      ),
      {
        x: [referencePayload.profile.x_m[0], comparePayload.profile.x_m[0]],
        y: [referencePayload.profile.y_m[0], comparePayload.profile.y_m[0]],
        mode: "markers",
        marker: { size: 10, color: "#ff4040", symbol: "x" },
        name: "Start (highlight)",
        hoverinfo: "skip",
        showlegend: false,
      },
    ];
    Plotly.newPlot(
      div,
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        margin: { l: 40, r: 88, t: 28, b: 40 },
        yaxis: { scaleanchor: "x", scaleratio: 1, title: "y (m)" },
        xaxis: { title: "x (m)" },
        showlegend: true,
        legend: { orientation: "h", y: 1.08, x: 0.02 },
        annotations: [
          {
            x: startArrow.tailX,
            y: startArrow.tailY,
            xref: "x",
            yref: "y",
            text: "Start",
            showarrow: false,
            xanchor: "left",
            yanchor: "bottom",
            xshift: 6,
            yshift: 6,
            font: { color: "#f5f7fa", size: 12 },
          },
          {
            x: startArrow.headX,
            y: startArrow.headY,
            ax: startArrow.tailX,
            ay: startArrow.tailY,
            xref: "x",
            yref: "y",
            axref: "x",
            ayref: "y",
            arrowhead: 3,
            arrowsize: 1.3,
            arrowwidth: 2.2,
            arrowcolor: "#f5f7fa",
            text: "",
          },
        ],
      }),
      { responsive: true },
    );
  }

  function renderSingleComparisonMap(plotId, payload, method, speedRange) {
    const div = document.getElementById(plotId);
    if (!div || !window.Plotly) return;
    const figure = buildTrackMapFigure(payload, speedRange, {
      title: formatMethodLabel(method),
      showLegend: true,
      showColorBar: true,
      colorBarTitle: "Speed (m/s)",
      margin: { l: 40, r: 58, t: 42, b: 40 },
    });
    Plotly.newPlot(
      div,
      figure.traces,
      figure.layout,
      { responsive: true },
    );
  }

  function bindMapZoomSync(firstId, secondId) {
    const first = document.getElementById(firstId);
    const second = document.getElementById(secondId);
    if (!first || !second || first.dataset.zoomSyncBound === "1") return;
    first.dataset.zoomSyncBound = "1";
    second.dataset.zoomSyncBound = "1";
    let syncing = false;
    const keys = ["xaxis.range[0]", "xaxis.range[1]", "yaxis.range[0]", "yaxis.range[1]", "xaxis.autorange", "yaxis.autorange"];

    function relay(source, target, event) {
      if (syncing || !event) return;
      const hasAxisChange = keys.some((key) => Object.prototype.hasOwnProperty.call(event, key));
      if (!hasAxisChange) return;
      syncing = true;
      const sourceXRange = source.layout?.xaxis?.range;
      const sourceYRange = source.layout?.yaxis?.range;
      const update = {};
      if (Array.isArray(sourceXRange) && sourceXRange.length === 2) {
        update["xaxis.range[0]"] = sourceXRange[0];
        update["xaxis.range[1]"] = sourceXRange[1];
      }
      if (Array.isArray(sourceYRange) && sourceYRange.length === 2) {
        update["yaxis.range[0]"] = sourceYRange[0];
        update["yaxis.range[1]"] = sourceYRange[1];
      }
      Plotly.relayout(target, update)
        .catch(() => undefined)
        .finally(() => {
          syncing = false;
        });
    }

    first.on("plotly_relayout", (event) => relay(first, second, event));
    second.on("plotly_relayout", (event) => relay(second, first, event));
  }

  function renderSpeedComparison(referencePayload, comparePayload) {
    const div = document.getElementById("plot-speed-comparison");
    if (!div || !window.Plotly) return;
    const traces = [referencePayload, comparePayload].map((payload) => ({
      x: payload.speed.s_nodes_m,
      y: payload.speed.final_speed_mps,
      mode: "lines",
      line: { color: methodColor(payload.method), width: 2.2 },
      name: formatMethodLabel(payload.method),
    }));
    Plotly.newPlot(
      div,
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Speed (m/s)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderLongAccelComparison(referencePayload, comparePayload) {
    const div = document.getElementById("plot-long-accel-comparison");
    if (!div || !window.Plotly) return;
    const traces = [
      {
        x: referencePayload.speed.s_midpoints_m,
        y: referencePayload.speed.forward_longitudinal_limit_mps2,
        mode: "lines",
        line: { color: "#4caf50", dash: "dot", width: 1.2 },
        name: "Drive limit",
      },
      {
        x: referencePayload.speed.s_midpoints_m,
        y: referencePayload.speed.braking_longitudinal_limit_mps2.map((value) => -value),
        mode: "lines",
        line: { color: "#e57373", dash: "dot", width: 1.2 },
        name: "Brake limit",
      },
      ...[referencePayload, comparePayload].map((payload) => ({
        x: payload.speed.s_midpoints_m,
        y: payload.speed.longitudinal_accel_mps2,
        mode: "lines",
        line: { color: methodColor(payload.method), width: 2.1 },
        name: formatMethodLabel(payload.method),
      })),
    ];
    Plotly.newPlot(
      div,
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Longitudinal a (m/s²)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderCurvatureComparison(referencePayload, comparePayload) {
    const div = document.getElementById("plot-curvature-comparison");
    if (!div || !window.Plotly) return;
    const traces = [referencePayload, comparePayload].map((payload) => ({
      x: payload.profile.s_m,
      y: payload.profile.curvature_1_per_m,
      mode: "lines",
      line: { color: methodColor(payload.method), width: 1.8 },
      name: formatMethodLabel(payload.method),
    }));
    Plotly.newPlot(
      div,
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Curvature (1/m)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderMetricCards(payload) {
    const container = document.getElementById("metric-cards");
    const m = payload.metrics;
    const cards = [
      ["Lap time", formatNumber(m.performance.lap_time_s), "s"],
      ["Avg speed", formatNumber(m.performance.average_speed_mps), "m/s"],
      ["Max speed", formatNumber(m.performance.max_speed_mps), "m/s"],
      ["Min speed", formatNumber(m.performance.min_speed_mps), "m/s"],
      ["Lap length", formatNumber(m.geometry.total_length_m), "m"],
      ["Closure gap", formatNumber(m.geometry.closure_gap_m, 4), "m"],
      ["κ rms", m.geometry.rms_curvature.toExponential(2), "1/m"],
      ["Corner severity", formatNumber(m.geometry.corner_severity_index, 5), ""],
      ["|aₓ| mean", formatNumber(m.performance.mean_abs_long_accel_mps2), "m/s²"],
      ["Lat-limit fraction", formatNumber(m.performance.lateral_limit_fraction * 100, 1), "%"],
      ["Accel / coast / brake", `${(m.performance.accel_fraction*100).toFixed(0)} / ${(m.performance.coast_fraction*100).toFixed(0)} / ${(m.performance.brake_fraction*100).toFixed(0)}`, "%"],
      ["Phase transitions", m.performance.phase_transition_count, ""],
    ];
    container.innerHTML = cards
      .map(
        ([label, value, unit]) =>
          `<div class="metric-card"><div class="label">${label}</div>` +
          `<div class="value">${value}<span class="unit">${unit}</span></div></div>`,
      )
      .join("");
  }

  function renderTrackMap(payload, speedRange) {
    const figure = buildTrackMapFigure(payload, speedRange, {
      showLegend: true,
      showColorBar: true,
      colorBarTitle: "Speed (m/s)",
      margin: { l: 40, r: 28, t: 28, b: 40 },
    });
    Plotly.newPlot(
      "plot-track-map",
      figure.traces,
      figure.layout,
      { responsive: true },
    );
  }

  function renderSpeedPlot(payload) {
    const s = payload.speed.s_nodes_m;
    const traces = [
      {
        x: s,
        y: payload.speed.speed_cap_mps,
        mode: "lines",
        line: { color: "#ff8a65", dash: "dash" },
        name: "Lateral cap",
      },
      {
        x: s,
        y: payload.speed.forward_speed_mps,
        mode: "lines",
        line: { color: "#ffd166", dash: "dot" },
        name: "Forward pass",
      },
      {
        x: s,
        y: payload.speed.final_speed_mps,
        mode: "lines",
        line: { color: "#4cc2ff", width: 2 },
        name: "Final",
      },
    ];
    Plotly.newPlot(
      "plot-speed",
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Speed (m/s)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderLongAccelPlot(payload) {
    const sMid = payload.speed.s_midpoints_m;
    const longAccel = payload.speed.longitudinal_accel_mps2;
    const fwdLimit = payload.speed.forward_longitudinal_limit_mps2;
    const brakeLimit = payload.speed.braking_longitudinal_limit_mps2.map((v) => -v);
    const traces = [
      {
        x: sMid,
        y: fwdLimit,
        mode: "lines",
        line: { color: "#4caf50", dash: "dot" },
        name: "Drive limit",
      },
      {
        x: sMid,
        y: brakeLimit,
        mode: "lines",
        line: { color: "#e57373", dash: "dot" },
        name: "Brake limit",
      },
      {
        x: sMid,
        y: longAccel,
        mode: "lines",
        line: { color: "#4cc2ff", width: 2 },
        name: "Applied aₓ",
      },
    ];
    Plotly.newPlot(
      "plot-long-accel",
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Longitudinal a (m/s²)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderCurvaturePlot(payload) {
    const traces = [
      {
        x: payload.profile.s_m,
        y: payload.profile.curvature_1_per_m,
        mode: "lines",
        line: { color: "#bb86fc", width: 1.5 },
        name: "κ (1/m)",
      },
    ];
    Plotly.newPlot(
      "plot-curvature",
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        xaxis: { title: "Arc length s (m)" },
        yaxis: { title: "Curvature (1/m)" },
        hovermode: "x unified",
      }),
      { responsive: true },
    );
  }

  function renderNumerics(payload) {
    const lines = [
      "Integration (lap time):",
      `  kinematic    ${formatNumber(payload.integration.kinematic_time_s)} s`,
      `  left rule    ${formatNumber(payload.integration.left_rule_time_s)} s`,
      `  trapezoidal  ${formatNumber(payload.integration.trapezoidal_time_s)} s`,
      `  Simpson      ${payload.integration.simpson_time_s == null ? "n/a" : formatNumber(payload.integration.simpson_time_s) + " s"}`,
      "",
      "Residuals (constraint violations, m/s²):",
      `  lateral          ${payload.residuals.lateral_mps2.toExponential(2)}`,
      `  acceleration     ${payload.residuals.acceleration_mps2.toExponential(2)}`,
      `  braking          ${payload.residuals.braking_mps2.toExponential(2)}`,
      `  friction circle  ${payload.residuals.friction_circle_mps2.toExponential(2)}`,
      "",
      `Audit: points=${payload.audit.point_count}, closure_gap=${payload.audit.closure_gap_m.toFixed(4)} m,` +
        ` spacing_cv=${payload.audit.spacing_cv.toFixed(3)}, outliers=${payload.audit.outlier_segment_count}`,
    ];
    document.getElementById("numerics").textContent = lines.join("\n");
  }

  function attachLinkedHover(payload) {
    const xs = payload.profile.x_m;
    const ys = payload.profile.y_m;
    const mapDiv = document.getElementById("plot-track-map");
    if (!mapDiv || !xs.length) return;

    const highlightIndex = mapDiv.data.length - 1; // last trace = highlight marker

    function highlight(index) {
      if (index < 0 || index >= xs.length) return;
      Plotly.restyle(
        mapDiv,
        { x: [[xs[index]]], y: [[ys[index]]] },
        [highlightIndex],
      );
    }

    function nearestIndex(sValue, sArray) {
      let lo = 0;
      let hi = sArray.length - 1;
      while (lo + 1 < hi) {
        const mid = (lo + hi) >> 1;
        if (sArray[mid] <= sValue) lo = mid;
        else hi = mid;
      }
      return sArray[hi] - sValue < sValue - sArray[lo] ? hi : lo;
    }

    function bindHover(plotId, sArray) {
      const div = document.getElementById(plotId);
      if (!div) return;
      div.on("plotly_hover", (event) => {
        if (!event || !event.points || !event.points.length) return;
        const s = event.points[0].x;
        highlight(nearestIndex(s, sArray));
      });
    }

    bindHover("plot-speed", payload.speed.s_nodes_m);
    bindHover("plot-long-accel", payload.speed.s_midpoints_m);
    bindHover("plot-curvature", payload.profile.s_m);
  }

  window.dashboard = { renderOverview, renderTrackDetail };
})();
