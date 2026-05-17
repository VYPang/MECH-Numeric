# Tutorial Notes: The Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories

Based on the paper *A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories* by Nitin R. Kapania, John Subosits, and J. Christian Gerdes.

## 1. Purpose of the Paper

The paper studies the minimum-lap-time trajectory problem for a vehicle on a closed race track. In the full formulation, one would optimize throttle, braking, steering, and the path itself at the same time. That is a nonlinear optimal control problem and is usually too expensive for fast planning.

The main idea of the paper is to replace the full simultaneous optimization with an iterative two-step method:

1. Given a path, compute the fastest feasible speed profile.
2. Given that speed profile, update the path so that it becomes easier to drive fast.

This decomposition is the numerical core of the paper. It is not guaranteed to produce the global optimum, but it is much faster than solving the fully coupled problem directly.

## 2. Why the Problem Is Hard

Lap time is

$$
t = \int_0^L \frac{ds}{U_x(s)}
$$

where:

- $s$ is arc length along the driven path,
- $L$ is the total path length,
- $U_x(s)$ is the longitudinal speed.

This equation shows the conflict immediately:

- a shorter path reduces $L$,
- a smoother path reduces curvature and often allows larger $U_x(s)$.

Those two goals usually compete. Cutting distance is not always best if it forces large curvature and therefore large speed reduction in the corner.

The paper's approximation is to focus the path-update step on reducing curvature, because on a race track that usually helps lap time more than making the path slightly shorter.

## 3. Path Representation

Instead of storing the track directly as $(E,N)$ coordinates only, the paper parameterizes the reference path by arc length $s$ and curvature $K(s)$.

The path heading is recovered by

$$
\Psi_r(s) = \int_0^s K(z)\,dz
$$

and the Cartesian coordinates are reconstructed from

$$
E(s) = \int_0^s -\sin(\Psi_r(z))\,dz,
\qquad
N(s) = \int_0^s \cos(\Psi_r(z))\,dz.
$$

The road boundaries are stored as lateral distances from the reference path:

- $w_{in}(s)$: distance to the inside edge,
- $w_{out}(s)$: distance to the outside edge.

This representation is numerically convenient because it turns the track-boundary condition into a simple lateral bound.

## 4. Vehicle Model Used in the Algorithm

Section 3 defined a reference path by its arc length $s$, curvature $K(s)$, heading $\Psi_r(s)$, and boundary distances $w_{in}(s)$ and $w_{out}(s)$. That reference path is the baseline used in one iteration of the algorithm.

The role of the vehicle model in this section is not to simulate a final lap directly. Its role is to describe, in a physically meaningful way, how a candidate new racing line can move relative to the current reference path. In other words:

- Section 3 defines the path-centered coordinate system,
- Section 4 defines how the vehicle moves inside that coordinate system,
- the optimization step later uses those equations as constraints.

### 4.1 Path-Relative States

The paper uses a planar bicycle model for the path-update step. The important lateral states are:

- $e$: lateral deviation from the current reference path,
- $\Delta \Psi$: heading error relative to the reference-path tangent,
- $r$: yaw rate,
- $\beta$: sideslip angle.

The most important state for understanding the optimization is $e$.

- If $e = 0$, the candidate vehicle path lies exactly on the current reference path.
- If $e > 0$ or $e < 0$, the candidate path is shifted sideways relative to that reference path.

So $e$ is best interpreted as the lateral shift from the current path guess to the updated path guess. It does not mean the vehicle is leaving the track. The optimizer is only allowed to choose values of $e$ that keep the path inside the road boundaries.

The corresponding derivative

$$
\dot e = \frac{de}{dt}
$$

is the rate at which that lateral shift changes with time.

Similarly, $\Delta \Psi$ measures angular misalignment:

- if $\Delta \Psi = 0$, the vehicle heading matches the local tangent of the reference path,
- if $\Delta \Psi \neq 0$, the vehicle is pointed slightly inward or outward relative to the path direction.

### 4.2 Key Path-Relative Dynamics

The paper writes the path-relative kinematics as

$$
\dot e = U_x(\beta + \Delta\Psi),
\qquad
\dot{\Delta\Psi} = r - U_x K.
$$

These two equations are the bridge between the geometry of Section 3 and the optimization step.

The first equation,

$$
\dot e = U_x(\beta + \Delta\Psi),
$$

says that the lateral offset changes when the vehicle has forward speed $U_x$ and its motion is not perfectly aligned with the reference path. The term $\beta + \Delta\Psi$ is the effective small angle between the vehicle velocity direction and the reference-path tangent. So this equation can be read as:

- forward speed,
- multiplied by sideways pointing angle,
- gives sideways motion relative to the reference path.

If $\beta + \Delta\Psi \approx 0$, then $\dot e \approx 0$, so the candidate path stays near the current reference path. If $\beta + \Delta\Psi$ is nonzero, the candidate path moves laterally and $e$ changes.

The second equation,

$$
\dot{\Delta\Psi} = r - U_x K,
$$

compares two turning rates:

- $r$: how fast the vehicle itself is rotating,
- $U_x K$: how fast the reference path is rotating.

So $\Delta \Psi$ increases when the vehicle turns more or less than the path requires. This is what keeps track of whether the candidate path remains aligned with the reference curve.

### 4.3 Why These States Matter for Optimization

The algorithm you described is:

1. fix a reference path,
2. compute the best speed profile on that path,
3. update the path,
4. repeat.

Section 4 is what makes Step 3 physically meaningful.

The optimizer is not allowed to invent a new path arbitrarily. Instead, it must produce a path that is consistent with the bicycle-model dynamics. That is why the path-update step works with vehicle states such as $e$, $\Delta \Psi$, $r$, and $\beta$ rather than with geometry alone.

In particular:

- $e$ tells the optimizer how far the updated path shifts sideways from the current reference path,
- $\dot e$ tells how that shift is produced by speed and orientation,
- $\Delta \Psi$ tells how the vehicle heading differs from the path direction,
- the boundary distances $w_{in}$ and $w_{out}$ limit the allowable values of $e$.

So the reference path from Section 3 is necessary because it provides the local frame in which all of these states are defined. Without that frame, $e$ and $\Delta \Psi$ would have no meaning.

### 4.4 From Continuous Model to Optimization Constraints

For optimization, the paper augments the state with the absolute vehicle heading $\Psi$, so the state vector becomes

$$
x = [e,\ \Delta\Psi,\ r,\ \beta,\ \Psi]^T.
$$

After tire-force linearization, the continuous bicycle model is converted into a discrete affine, time-varying system,

$$
x_{k+1} = A_k x_k + B_k \delta_k + d_k,
$$

where $\delta_k$ is steering input.

This is the form actually used by the optimizer. The optimizer solves for a steering sequence and a state trajectory that satisfy these equations, while also respecting the track-width constraint

$$
w_{out,k} \le e_k \le w_{in,k}.
$$

Once the optimal offset profile $e_k^*$ is found, the paper reconstructs the updated path from that offset. So the output of the optimization is not just a steering history; it is also a physically feasible new racing line relative to the old reference path.

## 5. Step 1: Fastest Speed Profile on a Fixed Path

### 5.1 Basic Principle

If the path is fixed, then its curvature $K(s)$ is known. The first numerical task is to compute the largest feasible speed at each point without violating tire friction limits.

The tire friction circle is the governing constraint:

$$
F_{x,f}^2 + F_{y,f}^2 \le (\mu F_{z,f})^2,
\qquad
F_{x,r}^2 + F_{y,r}^2 \le (\mu F_{z,r})^2.
$$

This expresses the tradeoff between lateral tire force and longitudinal tire force:

- if the car is using a lot of lateral force to turn,
- then less longitudinal force is available for acceleration or braking.

### 5.2 The Three-Pass Speed Computation

The paper uses a three-pass forward-backward integration method.

#### Pass 1: Lateral-grip speed limit

First compute the steady-state maximum speed allowed by curvature alone. In the simplified case used in the paper,

$$
U_x(s) = \sqrt{\frac{\mu g}{|K(s)|}}.
$$

This is the speed ceiling coming purely from cornering grip. It gives a first estimate of where the car must slow down.

Interpretation:

- large curvature $|K|$ means a tight corner,
- tight corners force smaller allowable speed,
- low-curvature regions allow larger speed.

#### Pass 2: Forward acceleration pass

The second pass propagates speed forward using the available longitudinal force for acceleration:

$$
U_x(s+\Delta s) = \sqrt{U_x^2(s) + 2\frac{F_{x,accel,max}}{m}\Delta s}.
$$

At every point, this forward-propagated speed is compared against the lateral-grip ceiling from Pass 1, and the smaller value is kept.

This captures the fact that even if the road ahead is straight, the car cannot jump instantly to a higher speed. It must build speed through finite acceleration.

#### Pass 3: Backward braking pass

The third pass propagates speed backward using the available braking force:

$$
U_x(s-\Delta s) = \sqrt{U_x^2(s) - 2\frac{F_{x,decel,max}}{m}\Delta s}.
$$

Again, pointwise minima are taken with the profile obtained so far.

This is what places the braking zones correctly. A corner may require low speed at its entry, and that requirement propagates backward along the preceding straight.

### 5.3 Why This Step Works Numerically

This first step is efficient because it avoids solving a global nonlinear program. Instead, it uses local physical constraints and propagation:

- the first pass gives the cornering-speed envelope,
- the second pass enforces achievable acceleration,
- the third pass enforces achievable deceleration.

The result is a minimum-time feasible speed profile for the current path, within the fidelity of the tire model and the path discretization.

## 6. Step 2: Update the Path for the Fixed Speed Profile

### 6.1 Core Idea

Once $U_x(s)$ is fixed, the paper does not directly re-solve for minimum lap time. Instead, it searches for a new path that reduces curvature while staying dynamically feasible and within track boundaries.

This is an approximation, not an exact statement that curvature alone is the objective. The reason this works is the tradeoff discussed earlier:

- a smoother path can support higher speed because the vehicle experiences less lateral demand,
- but a smoother path may also be slightly longer in arc length.

The algorithm chooses to emphasize curvature reduction because, on many race tracks, the gain from higher corner speed is more important than the small increase in path length.

The heuristic is:

> a lower-curvature path is usually a faster path,
> because lower curvature permits higher corner speed.

This is the paper's key approximation.

### 6.2 Tire Linearization and Affine Dynamics

Near the current operating condition, the nonlinear tire model is linearized. That produces an affine, time-varying model of the form

$$
x_{k+1} = A_k x_k + B_k \delta_k + d_k.
$$

This matters because convex optimization requires a problem structure with linear equality constraints and convex cost.

The important point is that the matrices $A_k$, $B_k$, and $d_k$ depend on the fixed speed profile from Step 1. So even though curvature itself is geometric, the path update is not purely a geometry problem. It is a constrained vehicle-motion problem carried out at the speed level that was just computed.

That is why the algorithm needs $U_x(s)$ before updating the path:

- the speed profile determines the operating condition at each point of the track,
- that operating condition changes the linearized vehicle dynamics,
- those dynamics determine which low-curvature paths are physically feasible.

### 6.3 The Optimization Problem

The paper formulates the path update as

$$
\min_{\delta, x}
\sum_k
\left(\frac{\Psi_k - \Psi_{k-1}}{s_k - s_{k-1}}\right)^2
+ \lambda (\delta_k - \delta_{k-1})^2
$$

subject to

$$
x_{k+1} = A_k x_k + B_k\delta_k + d_k,
$$

$$
w_{out,k} \le e_k \le w_{in,k},
$$

and for a closed circuit,

$$
x_1 = x_T.
$$

Interpretation of each term:

- The first term minimizes the squared curvature of the driven path because curvature is approximately heading change per unit arc length.
- The second term regularizes steering, preventing a numerically jagged or experimentally unrealistic steering sequence.
- The dynamic constraint enforces path-relative vehicle feasibility.
- The boundary constraint keeps the vehicle within the track.
- The periodic constraint makes the racing line close smoothly around the full lap.

### 6.4 What the Optimizer Is Actually Choosing

The optimizer does not directly choose a new curve $K(s)$.

Instead, it computes:

- an optimal steering sequence $\delta_k^*$,
- the resulting optimal states $x_k^*$,
- especially the optimal lateral offsets $e_k^*$ and headings $\Psi_k^*$.

Those optimal offsets define the new racing line relative to the old one.

So the optimizer is not solving the abstract problem "find the smoothest curve inside the track." It is solving the more specific problem

> find a lower-curvature path that the modeled vehicle can actually follow at the fixed speed profile.

This is the reason the speed solve comes before the path update in every iteration.

## 7. How the New Path Is Reconstructed

After solving the convex problem, the updated path in Cartesian coordinates is rebuilt from the optimal lateral deviation:

$$
E_k^{new} = E_k - e_k^* \cos(\Psi_{r,k}),
$$

$$
N_k^{new} = N_k - e_k^* \sin(\Psi_{r,k}).
$$

Then the new arc length and curvature are recomputed numerically:

$$
s_k = s_{k-1} + \sqrt{(E_k-E_{k-1})^2 + (N_k-N_{k-1})^2},
$$

$$
K_k = \frac{\Psi_k^* - \Psi_{k-1}^*}{s_k - s_{k-1}}.
$$

This updated path becomes the input to the next speed-profile computation.

## 8. Full Iterative Algorithm

The complete algorithm is:

1. Start from an initial path, often the track centerline.
2. Compute the fastest feasible speed profile on that path using the three-pass method.
3. Solve the convex minimum-curvature path-update problem using that speed profile.
4. Reconstruct the updated path and recompute its curvature and boundary distances.
5. Recompute the lap time.
6. Repeat until lap-time improvement becomes smaller than a tolerance.

Symbolically,

$$
\text{path}^{(i)}
\xrightarrow{\text{speed solve}}
U_x^{(i)}(s)
\xrightarrow{\text{curvature minimization}}
\text{path}^{(i+1)}.
$$

This diagram should be read carefully. The middle quantity $U_x^{(i)}(s)$ is not just a byproduct used to compute lap time. It is also an input to the next optimization subproblem. The path-update step uses that speed profile inside the vehicle model, so a more explicit reading is

$$
	\text{path}^{(i)}
\rightarrow
	\text{feasible speed profile } U_x^{(i)}(s)
\rightarrow
	\text{physics-constrained low-curvature path } \text{path}^{(i+1)}.
$$

In other words:

- curvature tells us what shape would be geometrically attractive,
- the speed profile tells us at what operating condition the car is expected to drive that shape,
- the vehicle model tells us whether that shape is dynamically feasible at that operating condition.

The paper reports that on the Thunderhill race circuit, the method converges in only a few iterations, which is the main practical strength of the approach.

## 9. How the Apex Is Located in This Framework

The paper does not introduce the apex as a separate optimization variable. Instead, the apex emerges from the optimized path.

For a given corner, the apex is the point where the optimized racing line comes closest to the inside edge of the track. A practical numerical definition is

$$
k_{apex} = \arg\min_{k \in \mathcal{C}} d_{in,k},
$$

where:

- $\mathcal{C}$ is the set of indices belonging to one corner,
- $d_{in,k}$ is the path-to-inside-boundary clearance at node $k$.

In words, once the path update is complete, the apex is found by scanning the corner and locating where the line reaches its deepest inside position.

Why this happens naturally:

- entering wide gives a larger effective turning radius,
- touching or approaching the inside near mid-corner reduces peak curvature,
- exiting wide allows the heading to unwind smoothly,
- lower curvature raises the admissible speed through the corner.

So the familiar outside-inside-outside racing line is not imposed explicitly. It is a consequence of minimizing curvature subject to vehicle dynamics and road boundaries.

## 10. Numerical Interpretation of the Two-Step Method

The algorithm can be interpreted as a coordinate-descent-style procedure on a difficult coupled problem.

In the full minimum-time problem, path and speed are tightly coupled:

- path determines curvature,
- curvature determines lateral force demand,
- lateral force demand changes the speed envelope,
- speed changes the path-update dynamics.

The paper breaks this loop into two easier subproblems:

1. optimize speed while holding path fixed,
2. optimize path while holding speed fixed.

That is why the method is fast enough to be practical, even though it is only approximate.

## 11. What the Method Gains and What It Gives Up

### Advantages

- It is much faster than full nonlinear optimal control.
- The speed-profile step is physically interpretable.
- The path-update step is convex after linearization, so it is numerically reliable and efficient.
- The method reproduces realistic racing behavior, including boundary usage and plausible apex placement.

### Limitations

- There is no guarantee of global optimality.
- Minimizing curvature is only a surrogate for minimizing lap time.
- The path update depends on linearized tire dynamics around the current operating point.
- The quality of the solution depends on the initial path, discretization, and model fidelity.

## 12. High-Level Summary

The paper's numerical algorithm is best understood as an alternating loop between physics and geometry:

- physics step: given a path, compute the fastest speed profile allowed by friction, acceleration, and braking,
- geometry step: given that speed profile, compute a smoother, lower-curvature path that still fits inside the track and respects vehicle dynamics.

This is why the algorithm is both fast and effective. It does not attack the full minimum-time problem directly. Instead, it repeatedly improves two coupled pieces of the problem until lap time stops improving.

For course-project purposes, this paper is important because it gives a clear numerical architecture:

- path representation,
- curvature computation,
- forward/backward speed propagation,
- constrained path optimization,
- iterative refinement.

That architecture is often more valuable educationally than a black-box nonlinear optimizer, because each step has a clear physical and numerical interpretation.