# Proposed Project Topic: Numerical Method Analysis for Low-Fidelity Racing Line and Speed Optimization

## 1. Proposed Direction

Instead of directly reproducing the high-fidelity vehicle model from Kapania, Subosits, and Gerdes, this project can use that paper as motivation and then build a course-appropriate numerical methods project around a simplified racing-line optimization problem.

The revised project would treat the car as a point mass moving along a two-dimensional track. The vehicle velocity is assumed to be tangent to the chosen path. Under this assumption, we do not model yaw rate, sideslip, steering dynamics, or nonlinear tire-force saturation. Instead, the physical model is reduced to a relationship between path curvature, allowable speed, acceleration limits, and lap time.

This is a better match for a numerical methods course because the main emphasis becomes:

- how the track and path are represented numerically,
- how curvature, arc length, and lap time are approximated,
- how a speed profile is computed under constraints,
- how different optimization methods behave,
- how numerical accuracy and computational cost trade off.

The paper remains useful as a conceptual reference, but the project goal is no longer to reproduce its full vehicle-dynamics formulation.

## 2. Proposed Title

**Numerical Method Analysis for Two-Dimensional Racing Line and Speed Optimization Using a Point-Mass Vehicle Model**

An alternative shorter title is:

**Low-Fidelity Racing Line Optimization as a Numerical Methods Study**

## 3. Main Research Question

Given a two-dimensional race track with finite width, how can we numerically compute a feasible racing path and speed profile that approximately minimize lap time, and how do different numerical methods affect accuracy, stability, and computational cost?

The project should not only output an optimized path. It should also explain why particular numerical methods were chosen and validate whether those choices are appropriate.

## 4. Simplified Physical Model

### 4.1 Point-Mass Assumption

The vehicle is modeled as a point mass moving along a path in the plane. The velocity direction is assumed to be tangent to the path.

This means the vehicle state can be simplified to quantities along the path:

- path position,
- speed $v$,
- path curvature $\kappa$,
- elapsed time $t$,
- optional remaining energy $E$.

This removes the need for a bicycle model, yaw rate, sideslip angle, and steering-state dynamics.

### 4.2 Path Representation

Let the track centerline be represented by a smooth curve

$$
c(u) = (x_c(u), y_c(u)),
$$

where $u$ is a track parameter. A candidate racing line can be represented by a lateral offset from the centerline:

$$
p(u) = c(u) + e(u)n(u),
$$

where:

- $p(u)$ is the candidate vehicle path,
- $e(u)$ is the lateral offset from the centerline,
- $n(u)$ is the unit normal vector of the centerline.

The track boundary constraint becomes

$$
e_{min}(u) \le e(u) \le e_{max}(u).
$$

In implementation, $e(u)$ can be discretized as

$$
e_0, e_1, \ldots, e_N.
$$

These offset values become the main path-design variables.

### 4.3 Curvature and Speed Constraint

For a planar curve $p(u) = (x(u), y(u))$, curvature is

$$
\kappa(u) =
\frac{x'(u)y''(u)-y'(u)x''(u)}{\left(x'(u)^2+y'(u)^2\right)^{3/2}}.
$$

Because the car is assumed to move tangent to the path, the required lateral acceleration is

$$
a_y = v^2 |\kappa|.
$$

If the maximum lateral acceleration is $a_{y,max}$, then

$$
v(u) \le \sqrt{\frac{a_{y,max}}{|\kappa(u)|+\epsilon}},
$$

where $\epsilon$ is a small value to avoid division by zero on straight segments.

This is the main low-fidelity physics link between path geometry and velocity.

### 4.4 Longitudinal Acceleration Constraint

The longitudinal speed profile can be constrained by

$$
\frac{d(v^2)}{ds} = 2a_x,
$$

where:

- $s$ is arc length along the candidate path,
- $a_x$ is longitudinal acceleration,
- $a_{x,min} \le a_x \le a_{x,max}$.

This constraint prevents the car from instantly jumping from low speed to high speed, and it naturally creates braking zones before corners and acceleration zones after corners.

### 4.5 Lap Time Objective

The lap time is

$$
T = \int_0^L \frac{ds}{v(s)}.
$$

In a discretized calculation, this becomes approximately

$$
T \approx \sum_{i=0}^{N-1} \frac{\Delta s_i}{v_i}.
$$

The direct optimization goal can therefore be stated as

$$
\min_{e_i, v_i} T(e,v)
$$

subject to:

$$
e_{min,i} \le e_i \le e_{max,i},
$$

$$
v_i^2 |\kappa_i(e)| \le a_{y,max},
$$

$$
a_{x,min} \le \frac{v_{i+1}^2-v_i^2}{2\Delta s_i} \le a_{x,max}.
$$

This is lower fidelity than the paper, but it directly targets lap time and is more appropriate for numerical method analysis.

## 5. Two Possible Optimization Formulations

### 5.1 Sequential Formulation

The first formulation keeps the spirit of the paper but uses the simplified point-mass model. The key idea is to treat the speed-profile computation as an inner solver and the path update as an outer optimization.

The design variables are only the path offsets:

$$
e = [e_0, e_1, \ldots, e_N]^T.
$$

For a fixed offset vector $e$, the following quantities are computed deterministically:

1. reconstruct the path $p_i(e)$,
2. compute segment lengths $\Delta s_i(e)$,
3. compute curvature $\kappa_i(e)$,
4. compute the lateral speed cap

$$
v_{lat,i}(e) = \sqrt{\frac{a_{y,max}}{|\kappa_i(e)| + \epsilon}},
$$

5. run a forward pass for acceleration feasibility,
6. run a backward pass for braking feasibility,
7. obtain the feasible speed profile $v_i(e)$,
8. evaluate lap time

$$
T(e) = \sum_{i=0}^{N-1} \frac{\Delta s_i(e)}{v_i(e)}.
$$

So the outer optimization problem is actually

$$
\min_e T(e)
$$

subject to

$$
e_{min,i} \le e_i \le e_{max,i}.
$$

In practice, it is also helpful to add a path-smoothness regularization term,

$$
J(e) = T(e) + \lambda_s \sum_{i=1}^{N-1} (e_{i+1} - 2e_i + e_{i-1})^2,
$$

so that the optimizer does not create highly oscillatory paths that are numerically unstable when curvature is computed.

#### Proposed Algorithm for 5.1

The most practical algorithm for this sequential formulation is **projected finite-difference gradient descent**, with **coordinate search** as a comparison direct method.

##### Option A: Projected finite-difference gradient descent

At iteration $k$:

1. Start from the current path offsets $e^{(k)}$.
2. Compute the feasible speed profile $v(e^{(k)})$ by the inner speed solver.
3. Compute the objective $J(e^{(k)})$.
4. Approximate the gradient numerically:

$$
\frac{\partial J}{\partial e_i}
\approx
\frac{J(e^{(k)} + h\,\mathbf{e}_i) - J(e^{(k)} - h\,\mathbf{e}_i)}{2h},
$$

where $\mathbf{e}_i$ is the coordinate basis vector and $h$ is a finite-difference step.

5. Update by gradient descent:

$$
e_{temp}^{(k+1)} = e^{(k)} - \alpha_k \nabla J(e^{(k)}).
$$

6. Project back into the track bounds:

$$
e_i^{(k+1)} = \min\{e_{max,i},\max(e_{min,i},e_{temp,i}^{(k+1)})\}.
$$

7. Recompute the speed profile and lap time.
8. Stop when the objective decrease or step norm becomes small.

This method is attractive for a numerical methods course because:

- the outer optimization is explicit and easy to explain,
- the gradient is computed numerically,
- one can study sensitivity to the finite-difference step $h$,
- one can compare line-search strategies for choosing $\alpha_k$.

##### Option B: Coordinate search or pattern search

As a direct-method comparison:

1. Start from an initial offset vector $e^{(0)}$.
2. For each node $i$, test perturbations $e_i \pm h$ within bounds.
3. Recompute the inner speed profile and objective each time.
4. Accept the perturbation that decreases $J$ the most.
5. Sweep over all nodes.
6. Reduce the step size $h$ when no improvement is found.
7. Stop when $h$ becomes sufficiently small.

This method requires more function evaluations but avoids numerical gradients. It provides a useful direct-method benchmark against projected gradient descent.

#### Inner Speed Solver Used in 5.1

The sequential formulation only works if the speed profile is recomputed every time the path changes.

For a fixed path:

1. Compute the lateral speed cap $v_{lat,i}$.
2. Forward acceleration pass:

$$
v^{f}_{i+1} = \min\left(v_{lat,i+1},\sqrt{(v^{f}_i)^2 + 2a_{x,max}\Delta s_i}\right).
$$

3. Backward braking pass with braking limit $b_{max} > 0$:

$$
v_i = \min\left(v^{f}_i,\sqrt{v_{i+1}^2 + 2b_{max}\Delta s_i}\right).
$$

This gives a feasible speed profile for that path under the simplified dynamics.

The modular structure is therefore

$$
e^{(k)}
\rightarrow
\kappa(e^{(k)})
\rightarrow
v(e^{(k)})
\rightarrow
J(e^{(k)})
\rightarrow
e^{(k+1)}.
$$

### 5.2 Direct Time Optimization

The second formulation optimizes lap time more directly. The design variables include both path offsets and speed values:

$$
z = [e_0, e_1, \ldots, e_N, v_0, v_1, \ldots, v_N]^T.
$$

In this case, the problem is not just a symbolic minimization. It is a constrained nonlinear program.

The objective is

$$
\min_z \; J(z)
=
\sum_{i=0}^{N-1} \frac{\Delta s_i(e)}{v_i}
+ \lambda_s \sum_{i=1}^{N-1}(e_{i+1}-2e_i+e_{i-1})^2.
$$

subject to the explicit constraints

$$
e_{min,i} \le e_i \le e_{max,i},
$$

$$
v_{min} \le v_i \le v_{max},
$$

$$
g_i^{lat}(z) = v_i^2 |\kappa_i(e)| - a_{y,max} \le 0,
$$

$$
g_i^{acc}(z) = v_{i+1}^2 - v_i^2 - 2a_{x,max}\Delta s_i(e) \le 0,
$$

$$
g_i^{brake}(z) = v_i^2 - v_{i+1}^2 - 2b_{max}\Delta s_i(e) \le 0.
$$

If the track is closed, one may also impose periodic boundary conditions on the path offsets, for example

$$
e_0 = e_N,
\qquad
e_1-e_0 = e_N-e_{N-1}.
$$

#### Proposed Algorithm for 5.2

The clearest algorithm for this direct formulation is **Sequential Quadratic Programming (SQP)** or an **SLSQP-type constrained optimizer**.

At each iteration, such a solver:

1. linearizes the nonlinear constraints,
2. builds a local quadratic approximation of the objective,
3. solves a constrained quadratic subproblem,
4. updates the variable vector $z$,
5. repeats until constraint violation and objective decrease are small.

This is appropriate because the direct formulation has:

- a nonlinear objective,
- nonlinear curvature constraints,
- bound constraints,
- acceleration/braking inequality constraints.

If implementing a full SQP solver is too ambitious, a practical alternative is:

- use an available constrained optimizer as the reference method,
- compare it against a self-implemented penalty or projected-gradient method.

#### Penalty-Method Alternative

If the course emphasis is on implementing methods yourselves, a simpler alternative is to convert the constrained problem into an unconstrained penalized objective:

$$
J_p(z) = J(z)
+ \rho_1 \sum_i \max(0, g_i^{lat}(z))^2
+ \rho_2 \sum_i \max(0, g_i^{acc}(z))^2
+ \rho_3 \sum_i \max(0, g_i^{brake}(z))^2.
$$

Then one can apply:

- gradient descent with numerical gradients,
- a direct search method,
- or a quasi-Newton method.

This is easier to implement, though it usually requires careful tuning of the penalty parameters $\rho_1, \rho_2, \rho_3$.

This approach is closer to direct numerical optimization and provides a good opportunity to compare optimization methods. It also makes the tradeoff between path length and speed explicit, because $\Delta s_i(e)$ and $v_i$ both depend on the chosen path.

### 5.3 How to Check Whether a New Trajectory Guess Is Valid

In the paper, trajectory validity is checked against the bicycle-model dynamics. In the simplified project, the validity check is easier because the model is lower fidelity.

For the point-mass model, a candidate path-and-speed pair is considered valid if all of the following hold:

1. **Track boundary constraint**

$$
e_{min,i} \le e_i \le e_{max,i}.
$$

2. **Tangency assumption**

The vehicle velocity is assumed tangent to the chosen path by construction. This is not something to verify afterward; it is part of the model definition. Once a path is defined, the velocity direction is taken to be its tangent direction.

3. **Lateral acceleration constraint**

$$
v_i^2 |\kappa_i(e)| \le a_{y,max}.
$$

This is the simplified replacement for the more detailed tire-force constraints in the paper.

4. **Longitudinal acceleration and braking constraints**

$$
a_{x,min} \le \frac{v_{i+1}^2-v_i^2}{2\Delta s_i} \le a_{x,max}.
$$

Equivalently, using braking magnitude $b_{max} = -a_{x,min} > 0$,

$$
v_{i+1}^2 - v_i^2 \le 2a_{x,max}\Delta s_i,
$$

$$
v_i^2 - v_{i+1}^2 \le 2b_{max}\Delta s_i.
$$

5. **Optional path smoothness requirement**

To avoid numerically meaningless zig-zag paths, one may require either:

- a bound on discrete second differences,
- or a regularization penalty in the objective.

#### Constraint Residual Check

Numerically, validity should be checked by reporting the maximum residual of each constraint family:

$$
r_{lat} = \max_i \left(v_i^2|\kappa_i| - a_{y,max}\right),
$$

$$
r_{acc} = \max_i \left(v_{i+1}^2 - v_i^2 - 2a_{x,max}\Delta s_i\right),
$$

$$
r_{brake} = \max_i \left(v_i^2 - v_{i+1}^2 - 2b_{max}\Delta s_i\right).
$$

A candidate solution is acceptable when these residuals are less than a chosen tolerance, for example

$$
r_{lat},\ r_{acc},\ r_{brake} \le 10^{-6}
$$

in nondimensionalized test problems, or a physically meaningful tolerance in dimensional variables.

So, in the simplified model, the answer to "does the next trajectory match the vehicle dynamics?" is:

> it matches the model if the path stays inside the track, the velocity is tangent by construction, and the computed speed profile satisfies the lateral and longitudinal acceleration inequalities everywhere.

This is the lower-fidelity analogue of the paper's dynamic feasibility constraint.

## 6. Numerical Methods to Analyze

A strong version of this project should not only present a final optimized racing line. It should compare numerical methods and justify which method is appropriate for each part of the computation.

### 6.1 Curve Fitting and Track Representation

Possible methods:

- piecewise linear interpolation,
- global polynomial interpolation,
- cubic spline interpolation,
- periodic cubic spline interpolation for closed tracks.

Analysis questions:

- Does the method produce a smooth centerline?
- Does it create artificial oscillations?
- Is curvature stable under refinement?
- How does the number of track points affect accuracy?

Validation examples:

- test on a circle, where exact curvature is known,
- test on a sinusoidal curve, where curvature varies smoothly,
- compare arc length and curvature convergence as the mesh is refined.

### 6.2 Numerical Differentiation for Curvature

Curvature requires first and second derivatives. This makes the problem sensitive to numerical differentiation.

Possible methods:

- finite difference derivatives,
- spline derivative evaluation,
- smoothed finite differences.

Analysis questions:

- Which method gives stable curvature?
- How sensitive is curvature to grid spacing?
- Does high-order differentiation amplify noise?

This part directly connects to the course topic of numerical differentiation and approximation error.

### 6.3 Numerical Integration for Arc Length and Lap Time

The project should explicitly compare integration methods for quantities such as

$$
L = \int ds,
\qquad
T = \int \frac{ds}{v(s)}.
$$

Possible methods:

- composite trapezoidal rule,
- Simpson's 1/3 rule,
- mixed Simpson and trapezoidal rules when the number of panels is not compatible,
- adaptive integration on high-curvature segments.

Analysis questions:

- Does a higher-order method always perform better?
- Are high-curvature regions responsible for most of the integration error?
- Can the curve be segmented so that expensive methods are used only where needed?
- How does integration error affect the final lap-time ranking?

This directly addresses the lecturer's expectation: the project should not simply use a formula, but should analyze when a method is accurate and when it is misleading.

### 6.4 Speed Propagation as an IVP-Like Computation

The speed profile can be computed using the relation

$$
\frac{d(v^2)}{ds} = 2a_x.
$$

This can be treated as a simple initial-value propagation problem along arc length.

Possible methods:

- direct kinematic update using $v_{i+1}^2 = v_i^2 + 2a_x\Delta s_i$,
- explicit Euler form for $dv/ds = a_x/v$,
- Runge-Kutta methods for more general acceleration models.

Analysis questions:

- Is the direct $v^2$ update more stable than integrating $v$ directly?
- How does step size influence braking-point prediction?
- Does a higher-order RK method matter when the acceleration model is piecewise constant?

This connects the project to IVP methods without requiring a complicated vehicle model.

### 6.5 Root Finding Opportunities

Root finding can be included naturally in post-processing and event detection.

Possible uses:

- locating apex points by finding extrema of lateral offset or distance to the inside boundary,
- locating braking points where forward and backward speed envelopes intersect,
- finding points where curvature changes sign,
- finding where acceleration switches from positive to braking.

Possible methods:

- bisection,
- secant method,
- Newton's method when derivatives are available.

Analysis questions:

- Which event functions are smooth enough for Newton's method?
- When is bisection more reliable?
- How accurately do event locations need to be found before lap time stops changing meaningfully?

### 6.6 Optimization Methods

This is likely the most important course connection.

Possible methods:

- coordinate search over lateral offsets,
- Nelder-Mead or pattern search as direct methods,
- finite-difference gradient descent,
- projected gradient descent with boundary constraints,
- penalty-function methods,
- constrained optimization as a reference solution.

Analysis questions:

- Which methods converge reliably from the centerline initial guess?
- Which methods are sensitive to initial conditions?
- How many function evaluations are required?
- Does a lower objective value always correspond to a physically better path?
- How should constraint violations be penalized or projected?

This gives the project a clear numerical-methods identity rather than making it only a vehicle simulation.

## 7. Validation Plan

The validation should be designed around numerical correctness, not only around whether the final plot looks realistic.

### 7.1 Geometry Validation

Use simple curves with known properties:

- straight line: curvature should be zero,
- circle: curvature should be constant $1/R$,
- clothoid-like or sinusoidal curve: curvature should vary smoothly.

Compare numerical curvature, arc length, and integration results against exact or high-resolution reference values.

### 7.2 Speed-Profile Validation

Check that the computed speed profile satisfies:

$$
v_i^2 |\kappa_i| \le a_{y,max},
$$

and

$$
a_{x,min} \le \frac{v_{i+1}^2-v_i^2}{2\Delta s_i} \le a_{x,max}.
$$

The project should report maximum constraint violation, not only lap time.

### 7.3 Optimization Validation

Compare several solutions:

- centerline baseline,
- manually chosen outside-inside-outside path,
- optimized path from direct search,
- optimized path from gradient-based method,
- optional constrained-solver reference result.

For each method, compare:

- lap time,
- path length,
- maximum curvature,
- number of function evaluations,
- runtime,
- constraint violation.

### 7.4 Grid Refinement Study

Run the same problem with increasing numbers of path nodes, for example

$$
N = 50, 100, 200, 400.
$$

The goal is to show whether the result converges as the discretization is refined.

This is especially important because a visually smooth path can still have inaccurate curvature or lap-time estimates.

## 8. Recommended Project Structure

### Stage 1: Track and Path Geometry

- Generate or input a two-dimensional track.
- Fit the centerline using several interpolation methods.
- Construct track boundaries.
- Represent candidate paths using lateral offsets.

### Stage 2: Curvature and Integration Study

- Compute curvature using multiple derivative methods.
- Compute arc length using multiple integration methods.
- Validate against known curves.
- Select a method based on accuracy and cost.

### Stage 3: Point-Mass Speed Profile

- Compute lateral speed limit from curvature.
- Add acceleration and braking constraints.
- Compare speed propagation methods and discretization sizes.

### Stage 4: Time Optimization

- Define the objective $T = \sum \Delta s_i/v_i$.
- Optimize path offsets using at least two optimization methods.
- Compare direct and gradient-based approaches.
- Report convergence behavior and constraint violations.

### Stage 5: Optional Energy Extension

If time allows, add a simple energy model:

$$
E_{i+1} = E_i - c_{drive}\max(a_{x,i},0)\Delta s_i
+ \eta_{regen}c_{brake}\max(-a_{x,i},0)\Delta s_i.
$$

This can turn the project into a multi-objective problem:

$$
J = T - \lambda E_{final},
$$

or

$$
J = w_T T + w_E(E_0-E_{final}).
$$

However, this should be treated as an extension after the time-optimization problem is working.

## 9. Course Topic Coverage

This revised project can naturally demonstrate many topics from the numerical methods course.

| Course topic | Role in project |
|---|---|
| Root finding | Apex detection, braking-point detection, curvature sign changes |
| Polynomial and curve fitting | Track centerline and path representation |
| Numerical differentiation | Curvature computation |
| Numerical integration | Arc length, lap time, energy use |
| IVP methods | Speed propagation along arc length |
| Matrix computation | Spline systems, finite-difference smoothing, constrained least-squares options |
| Optimization | Racing-line and speed-profile optimization |

PDE and FEM methods are less central to this project and do not need to be forced into the scope.

## 10. Why This Scope Is Better for the Course

This revised scope has two advantages.

First, it keeps the engineering problem complex enough to be interesting. The project still includes geometry, constraints, speed propagation, optimization, and validation.

Second, it gives more room to discuss numerical methods. Instead of spending most of the effort reproducing a sophisticated vehicle model, the project can focus on questions such as:

- Which interpolation method gives reliable curvature?
- Which integration method gives accurate lap time at reasonable cost?
- How sensitive is the optimized path to discretization?
- Which optimization method converges fastest and most reliably?
- How should constraints be handled numerically?

These are exactly the kinds of questions that fit a numerical methods final project.

## 11. Proposed Final Deliverables

The final project can include:

1. A mathematical formulation of the point-mass racing-line problem.
2. A comparison of track interpolation and curvature-computation methods.
3. A comparison of numerical integration methods for arc length and lap time.
4. A speed-profile solver with acceleration and braking constraints.
5. At least two optimization methods for path offsets.
6. Grid-refinement and constraint-violation analysis.
7. Plots of optimized path, curvature, speed profile, acceleration, and lap time convergence.
8. A discussion of why the chosen numerical methods are appropriate.

## 12. Summary Recommendation

The recommended project is not to replicate the paper's high-fidelity two-step vehicle model. Instead, the project should use the paper as inspiration and formulate a lower-fidelity but numerically rich problem:

> Find a time-minimizing racing line and speed profile for a point-mass vehicle on a two-dimensional track, then analyze how numerical choices in curve fitting, differentiation, integration, speed propagation, and optimization affect the final result.

This version is still connected to racing trajectory optimization, but it better matches the purpose of a numerical methods course. It also creates a stronger final report because the project can explain and justify numerical choices rather than simply applying one advanced algorithm from the literature.