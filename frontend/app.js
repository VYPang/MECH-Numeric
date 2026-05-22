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

  // ---------- Overview ----------

  function renderOverview() {
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
    status.textContent = "Loading rankings (first solve may take a moment)…";

    getJSON("/api/rankings?method=centerline_baseline")
      .then((payload) => {
        const tbody = document.querySelector("#rankings-table tbody");
        tbody.innerHTML = "";

        let rows = payload.rows.slice();
        let sortKey = "difficulty_score";
        let sortDir = -1;

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
              const href = `/track?name=${encodeURIComponent(row.track_name)}`;
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
              window.location.href = `/track?name=${encodeURIComponent(tr.dataset.track)}`;
            });
          });
        }

        document.querySelectorAll("#rankings-table th").forEach((th) => {
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

        rerender();
        status.textContent = `Loaded ${rows.length} tracks.`;
      })
      .catch((error) => {
        status.textContent = `Failed to load rankings: ${error.message}`;
      });
  }

  // ---------- Track detail ----------

  function renderTrackDetail() {
    const params = new URLSearchParams(window.location.search);
    const trackName = params.get("name");
    if (!trackName) {
      document.getElementById("track-title").textContent = "Missing track name";
      return;
    }

    document.getElementById("track-title").textContent = trackName;
    document.getElementById("track-subtitle").textContent = "Loading baseline solution…";

    getJSON(`/api/tracks/${encodeURIComponent(trackName)}/baseline`)
      .then((payload) => {
        document.getElementById("track-subtitle").textContent =
          `Method: ${payload.method} · Geometry: ${payload.geometry_method}`;
        renderMetricCards(payload);
        renderTrackMap(payload);
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

  function renderTrackMap(payload) {
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
          colorbar: { title: "Speed (m/s)", thickness: 14 },
        },
        text: nodeSpeed.map(
          (v, i) => `s=${formatNumber(payload.speed.s_nodes_m[i])} m<br>v=${formatNumber(v)} m/s`,
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
    Plotly.newPlot(
      "plot-track-map",
      traces,
      Object.assign({}, PLOT_LAYOUT_DEFAULTS, {
        margin: { l: 40, r: 28, t: 28, b: 40 },
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
