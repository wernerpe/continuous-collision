# The algorithm

This page condenses Sections II to IV of the paper
([certified_ccd_paper.pdf](certified_ccd_paper.pdf)) to what a reader needs to
trust the proof. Equation numbers in parentheses are the paper's.
Where the paper and the header documentation of Drake PR #24924 disagree, this
page follows the header and says so.

## Setting

The input is a Drake `RobotDiagram`: a `MultibodyPlant` coupled to the
`SceneGraph` that holds its geometry.
The plant has $n$ generalized positions, so a configuration is
$q \in \mathbb{R}^n$.
Every pair $p = (A, B)$ of proximity geometries that the model's collision
filters leave unfiltered must be kept apart.
At a configuration $q$, Drake reports the pair's signed distance
$\varphi_p(q)$.
The caller supplies one margin $\delta \ge 0$ in meters, uniform over pairs.
The goal is $\varphi_p(q(t)) > \delta$ for every pair and every time $t$.

## 1. What one distance query buys

Fix a pair $p = (A, B)$ and two configurations $q$ and $q'$.
Let $\Delta$ be any number such that no material point of $A$ travels farther
than $\Delta$ between $q$ and $q'$, measured in a frame rigidly attached to
the body carrying $B$. Then

$$\varphi_p(q') \;\ge\; \varphi_p(q) - \Delta. \tag{1}$$

Proof. Let $x'$ and $y'$ be the closest points of $A$ and $B$ at $q'$, and
let $x$ be the position at $q$ of the material point that sits at $x'$, all
in $B$'s frame. Then
$\lVert x' - y' \rVert \ge \lVert x - y' \rVert - \lVert x' - x \rVert \ge \varphi_p(q) - \Delta$.

So the pair still clears the margin at $q'$ whenever

$$\Delta \;<\; \varphi_p(q) - \delta. \tag{2}$$

The right-hand side is the *clearance surplus*: what one query at $q$ leaves
to spend on motion.

## 2. Bounding the motion one coordinate at a time

Travel from $q$ to $q'$ along the axis-aligned path that changes $q_1$, then
$q_2$, and so on. Displacements add along that path.

**Which coordinates matter.** Moving coordinate $j$ changes the pair's
relative pose only when exactly one of the pair's two bodies lies in the
subtree outboard of $j$'s joint:

$$J(p) = \{\, j : \text{exactly one body of } p \text{ is outboard of } j \,\}. \tag{3}$$

For $j \in J(p)$, the outboard body is the pair's *distal side* for $j$.
On a self-collision pair the distal side changes from coordinate to
coordinate.

**The control box.** $Q = \prod_j [l_j, u_j]$ is the smallest axis-aligned box
that holds every control point of every segment of the normalized trajectory
(Section 4).

**Motion constants.** Every material point of the distal side moves by at most
$\lambda(j,p)\,|\Delta q_j|$ when coordinate $j$ moves alone.
The motion constant (4) is in meters of displacement per radian or per meter
of coordinate change:

| joint coordinate | $\lambda(j,p)$ |
|---|---|
| revolute; planar rotation | $r(j,p)$ |
| prismatic; planar translation | $1$ |
| screw | $r(j,p) + \lvert h_j \rvert / 2\pi$ |
| weld | no coordinates |

$h_j$ is the screw pitch in meters per revolution.
For a revolute joint the exact chord bound is $2r\sin(|\Delta q_j|/2)$; the
checker uses the slightly larger $r\,|\Delta q_j|$.

**The reach.** $r(j,p)$ is the largest distance from the origin of joint $j$'s
outboard frame (the point that stays fixed while $q_j$ moves) to any point of
the distal side's proximity geometry, over all configurations in $Q$.
It is bounded without evaluating any configuration, by walking the kinematic
chain from the distal body up to joint $j$:

$$r(j,p) \;\le\; \rho(j,p) + \sum_i \bigl(\ell_i + \bar u_i\bigr). \tag{5}$$

$\rho(j,p)$ is the radius, about the distal body's frame origin, of a sphere
containing that body's proximity geometry.
The sum runs over the *hops* of the chain from that body up to joint $j$'s
outboard frame, one hop per joint in between.
$\ell_i$ is the sum of the norms of hop $i$'s two fixed frame offsets.
$\bar u_i$ bounds, over $Q$, the translation that hop $i$'s own coordinates
produce, so an unbounded prismatic joint stays usable and the bound tightens
to the commanded motion.
Rotations along the chain turn the offsets without changing their lengths, so
summing norms is sound by the triangle inequality.

**Bounding spheres.** $\rho$ is $\max_g (\lVert c_g \rVert + R_g)$ over the
body's geometries $g$, where $c_g$ and $R_g$ are the center (in the body frame)
and radius of a sphere containing $g$. Each formula is an exact containment
statement:

| shape | center | radius $R_g$ |
|---|---|---|
| `Sphere` (radius $R$) | sphere center | $R$ |
| `Box` (half-widths $a, b, c$) | box center | $\sqrt{a^2 + b^2 + c^2}$ |
| `Capsule` (radius $R$, length $L$) | capsule center | $L/2 + R$ |
| `Cylinder` (radius $R$, length $L$) | cylinder center | $\sqrt{R^2 + L^2/4}$ |
| `Ellipsoid` (semi-axes $a, b, c$) | ellipsoid center | $\max(a, b, c)$ |
| `Convex`, `Mesh` (hull vertices $v_i$) | centroid of the hull vertices | $\max_i \lVert v_i - \text{centroid} \rVert$ |
| `HalfSpace` | none | none |

The hull vertices come from the hull the proximity engine actually collides
(`Shape.GetConvexHull()`), never from the raw file.
A half space has no bounding sphere and no finite reach; Section 8 lists its
rules.

## 3. The free ball

Summing the single-coordinate steps gives the bound the checker runs on.

**Lemma 1 (Displacement).** Let $q, q' \in Q$, let $p$ be an unfiltered pair,
and suppose the right-hand side below is positive. Then

$$\varphi_p(q') \;\ge\; \varphi_p(q) - \sum_{j \in J(p)} \lambda(j,p)\,\lvert q'_j - q_j \rvert. \tag{6}$$

Proof sketch (paper, Appendix A). Move one coordinate of $J(p)$ at a time
along the axis-aligned path from $q$ to $q'$.
Every intermediate configuration stays in $Q$, because a box is closed under
changing one coordinate at a time, so each reach is valid at every step.
At the step that changes $q_j$, only the distal side moves relative to the
other body. Its relative transform factors as
$X_{OD} = X_{OP}\,X_{PF}\,X_{FM}(q_j)\,X_{MD}$, in which only $X_{FM}$ depends
on $q_j$.
A material point of the distal side, at distance at most $r(j,p)$ from the
joint's outboard frame origin, therefore moves by at most
$\lambda(j,p)\,|\Delta q_j|$, and (1) turns that into the step's loss of
clearance.
Summing the steps gives (6). The sum is positive, so the geometries stay
separated throughout, and no regularity of penetration depth is needed.

Putting (6) into (2): every configuration of $Q$ in

$$\mathcal{B}_p(q) = \Bigl\{\, q' \in \mathbb{R}^n : \sum_{j \in J(p)} \lambda(j,p)\,\lvert q'_j - q_j \rvert \;<\; \varphi_p(q) - \delta \,\Bigr\} \tag{7}$$

keeps pair $p$ more than $\delta$ apart.
This *free ball* is a weighted $\ell_1$ ball centered at $q$: a cross-polytope
whose semi-axis along coordinate $j$ is $(\varphi_p(q) - \delta)/\lambda(j,p)$.
The checker works one pair at a time, at one query per pair, and certifies the
part of $\mathcal{B}_p(q)$ that lies in $Q$.

Example (paper, Fig. 1). A planar two-link arm with $L_1 = L_2 = 0.5$ m and a
tip disk of radius 0.1 m measures $\varphi_p = 0.12$ m to an obstacle, with
$\delta = 0.02$ m. The reaches are $r(1,p) = 1.1$ m at the shoulder and
$r(2,p) = 0.6$ m at the elbow. The free ball is
$1.1\,|\Delta\theta_1| + 0.6\,|\Delta\theta_2| < 0.10$, with semi-axes of
5.2° and 9.5°.

Everything in (7) except $\varphi_p(q)$ is fixed before the check runs.
Construction computes the bounding spheres and the subtree tables that give
$J(p)$. Each call assembles the reaches, and with them the constants
$\lambda(j,p)$, against that trajectory's $Q$ into a sparse table.
Evaluating (7) then costs one distance query and one dot product over $J(p)$.

## 4. Covering a trajectory with balls

**Normalization.** Every accepted input is converted, exactly and up front,
into an ordered list of Bézier segments over the plant's positions.

- A waypoint matrix or an edge is a list of order-1 segments.
- `BezierCurve` control points are copied.
- `BsplineTrajectory` is converted by knot insertion (Boehm's algorithm) until
  every span carries a full set of control points.
- `PiecewisePolynomial` is converted by the monomial-to-Bernstein change of
  basis, $c_k = \sum_{i=0}^{k} \binom{k}{i}\binom{m}{i}^{-1} a_i$. The
  conversion is ill-conditioned as the degree grows, so it is capped at
  degree 10.

Adjacent segments must agree at their junction to within a *continuity
tolerance* of $10^{-7}$ per coordinate (modulo $2\pi$ for the coordinates in
`Options.continuous_revolute_indices`).
A certificate over segments that teleport would silently miss the jump.

**The carve-out.** A coordinate whose control points all agree to within
$10^{-7}$ is dropped from every $J(p)$ for this call.
It can still move by the width $u_j - l_j$ of its side of $Q$, so each pair is
charged the *carve-out residual*

$$\sigma_p = \sum_{j \in C(p)} \tilde\lambda(j,p)\,(u_j - l_j) \tag{9}$$

over the set $C(p)$ of coordinates dropped from $J(p)$.
$\tilde\lambda(j,p)$ is $\lambda(j,p)$ where (4) applies.
Otherwise it is $r(j,p)$ for a roll-pitch-yaw, ball or universal rotation, $1$
for a floating translation, and $2r(j,p)/\mu$ for a quaternion coefficient,
with $\mu$ the smallest quaternion norm over $Q$.
$\sigma_p$ is zero exactly when the carved coordinates are exactly constant,
the common case.
This is what lets a constant floating base or ball joint into the model.

**Nodes.** A node $\nu$ is a sub-curve of one segment over a parameter
interval, carried by its own control points $P_0, \dots, P_m$.
Splitting $\nu$ at its parameter midpoint by de Casteljau's algorithm returns
two children whose control points represent the two halves exactly.
The apex of the de Casteljau triangle is the curve's value at the midpoint.
That value is the node's *representative configuration* $q_c$, and it lies
exactly on the trajectory.

**The node's control box.** A Bézier curve never leaves the convex hull of its
control points. With the half-widths

$$w_j(\nu) = \max_k \bigl\lvert (P_k)_j - (q_c)_j \bigr\rvert, \tag{10}$$

$\lvert q_j(s) - (q_c)_j \rvert \le w_j(\nu)$ for every parameter $s$ in the
node. The whole node lies in its control box
$\prod_j [(q_c)_j - w_j(\nu),\, (q_c)_j + w_j(\nu)]$, which lies inside $Q$.

**One query per node and pair.** With the *motion bound*

$$\Delta_p(\nu) = \sigma_p + \sum_{j \in J(p)} \lambda(j,p)\,w_j(\nu), \tag{11}$$

the node's control box lies inside $\mathcal{B}_p(q_c)$, and pair $p$ is
certified over all of $\nu$, as soon as

$$\hat\varphi_p(q_c) - \tau_p - \Delta_p(\nu) \;>\; \delta + \varepsilon. \tag{12}$$

$\hat\varphi_p$ is the reported distance, $\tau_p$ the pair's oracle tolerance
and $\varepsilon$ the numerical slack (Section 5).

**A definite violation.** When (12) fails, the checker asks whether the node is
definitely bad:

$$\hat\varphi_p(q_c) + \tau_p \;<\; \delta \tag{13}$$

puts $q_c$ itself below the margin.
$q_c$ lies exactly on the trajectory, so this is a definite violation, not a
sampling artifact, and $q_c$ is the witness.

**Split.** Otherwise the pair is undecided on this node and moves on to both
children. Under bisection the half-widths shrink by a constant factor per
split, so the motion bound (11) shrinks geometrically with depth while
$\hat\varphi_p(q_c)$ converges to the clearance at the limiting parameter.
Both (12) and (13) therefore fire at a depth logarithmic in one over the
amount by which the trajectory clears or violates the margin.
Effort concentrates where the clearance is tight.

**The resolution floor.** A pair stops being refined on a node once its travel
bound $\sum_{j \in J(p)} \lambda(j,p)\,w_j(\nu)$ is at most
$r$ = `Options.distance_resolution` (default $10^{-6}$ m).
($r$ without arguments is this resolution; $r(j,p)$ is the reach.)
Only the travel term is tested, because $\sigma_p$ does not shrink under
splitting.
A pair still undecided there records an *inconclusive witness*: the node's
$q_c$, at which the trajectory grazes the margin to within the oracle
tolerance.
A floating-point backstop also stops a node whose midpoint no longer separates
its endpoints in double precision.

> The paper (Section III-B) describes an earlier floor, `min_interval`: a node
> narrower than $10^{-9}$ of its segment's parameter range is not split again.
> PR #24924 replaced it with the per-pair distance floor above (commit
> `c5bf9cda0b`). This package follows the PR.

**The verdict.** `kViolationFound` at the earliest recorded violation if there
is one; otherwise `kInconclusive` at the earliest inconclusive witness if
there is one; otherwise `kCertifiedFree`.

**Four cheap improvements.**

- *Prefilter.* The bounding spheres, posed at $q_c$, give
  $\varphi_p \ge \lVert c_A - c_B \rVert - R_A - R_B$.
  When that lower bound already satisfies (12), the pair is certified with no
  narrowphase work.
- *Drop.* A pair certified at a node is certified on every sub-curve of it, so
  it is dropped for the node's whole subtree. This is the dominant work-saver.
- *Breakpoints.* The loop queries interior midpoints only, so it reaches
  $q(t_0)$, the junctions and $q(t_f)$ only in the limit.
  Those finitely many configurations are checked directly, one query per pair.
  A pair with $J(p) = \emptyset$ has its relative pose fixed by (3), so the
  check at $t_0$ settles it for good.
- *Earliest violation.* A node is queried at its midpoint before its left half
  is, so the first violation found need not be the earliest.
  The loop records a violation instead of returning, keeps the earliest time
  $t^*$, and skips every node that begins at or after $t^*$.

The resulting loop, with the PR's resolution floor:

```
CERTIFY(segments, pairs, δ, r)
 1  check q(t0), every junction and q(tf) directly, one query per pair;
    settle the pairs with J(p) = ∅ at t0
 2  t* ← ∞; push each segment's root node with S0 = {p : J(p) ≠ ∅}, last segment first
 3  while the stack is not empty: pop (ν, S)
 4      if ν begins at or after t*: continue
 5      (ν_L, ν_R, q_c) ← DeCasteljau(ν) at time t_c;  w ← half-widths about q_c   (10)
 6      pose the plant at q_c;  S' ← ∅
 7      for each pair p in S:
 8          travel ← Σ_{j ∈ J(p)} λ(j,p)·w_j;  Δ ← σ_p + travel;  floor ← travel ≤ r   (11)
 9          if SphereBound(p, q_c) − τ_p − Δ > δ + ε: continue                      prefilter
10          φ̂ ← Oracle(p, q_c)
11          if φ̂ + τ_p < δ: record a violation at (t_c, q_c, p), lower t*;          (13)
                if floor: continue
12          elif φ̂ − τ_p − Δ > δ + ε: continue                                       (12)
13          elif floor: record an inconclusive witness at (t_c, q_c, p); continue
14          S' ← S' ∪ {p}
15      if S' ≠ ∅: push (ν_R, S'), then (ν_L, S')
16  return kViolationFound at the earliest recorded violation;
    else kInconclusive if a witness was recorded; else kCertifiedFree
```

Pushing the right child first makes the pop order left-first in time, which
tightens $t^*$ early.

## 5. Tolerances

**The oracle contract.** The oracle reports $\hat\varphi_p$ rather than
$\varphi_p$, to a per-pair tolerance $\tau_p$: whenever
$\varphi_p(q) \ge -\tau_p$,

$$\bigl\lvert \hat\varphi_p(q) - \varphi_p(q) \bigr\rvert \;\le\; \tau_p, \tag{8}$$

and below that it reports a definitely negative value.

**$\tau_p$.** $\tau_p = \max(10^{-6}\ \text{m},\ \text{documented accuracy})$,
where the documented accuracy is the one Drake states for
`ComputeSignedDistancePairClosestPoints` for that shape combination (Table 4
of `drake/geometry/query_object.h`).
`Mesh` is certified as its convex hull, so it carries `Convex`'s values.
Pairs with a half space are computed in closed form and carry the default
$10^{-6}$ m.
Every combination not listed below is documented at $6 \times 10^{-15}$ m or
better and carries the default.

| shape pair | documented accuracy (m) |
|---|---|
| Box–Capsule | $3 \times 10^{-6}$ |
| Sphere–Convex, Sphere–Mesh | $3 \times 10^{-6}$ |
| Capsule–Ellipsoid | $5 \times 10^{-6}$ |
| Box–Cylinder, Cylinder–Convex, Cylinder–Mesh | $6 \times 10^{-6}$ |
| Box–Ellipsoid, Ellipsoid–Convex, Ellipsoid–Mesh | $9 \times 10^{-6}$ |
| Capsule–Cylinder | $1 \times 10^{-5}$ |
| Capsule–Capsule, Capsule–Convex, Capsule–Mesh | $2 \times 10^{-5}$ |
| Cylinder–Cylinder, Ellipsoid–Ellipsoid | $2 \times 10^{-5}$ |
| Sphere–Ellipsoid | $4 \times 10^{-5}$ |
| Cylinder–Ellipsoid | $5 \times 10^{-5}$ |

**$\varepsilon$.** Every certificate test carries an internal slack
$\varepsilon = 10^{-9}$ m (1 nm). It dominates the accumulated floating-point
error of the $w$, $\lambda$ and dot-product expressions.
The library does not use directed rounding.

**What an oracle failure can do.** A certified node is contradicted only if
some pair's true clearance dips to the margin on it.
The argument from $q_c$ to the first such crossing involves only clearances at
or above the margin.
Oracle misbehavior below the margin therefore cannot fake a certificate.

## 6. The guarantee

If a check returns `kCertifiedFree`, then for every time $t$ in the
trajectory's domain and every unfiltered geometry pair $(A, B)$, the signed
distance $\varphi_{AB}(q(t))$ exceeds `Options.margin`.
That holds under three assumptions:

1. exact real arithmetic up to the internal numerical slack $\varepsilon$
   (1 nm);
2. a distance oracle accurate to its stated tolerance $\tau_p$, as in (8);
3. `Mesh` ≡ convex hull.

A capability probe at construction raises, naming the geometries, if any
shape combination falls outside these assumptions.
The certificate is a statement about the model.
Transferring it to hardware also needs a collision model that
over-approximates the physical robot and a tracking error that fits inside the
margin.

### The resolution contract

Write $\delta$ for `Options.margin`, $r$ for `Options.distance_resolution`,
$\tau_p$ for the oracle tolerance of pair $p$ (at least 1 µm; Drake's
documented signed-distance accuracy for that shape combination), $\varepsilon$
for the internal slack (1 nm), and $\sigma_p$ for the residual motion of
coordinates the trajectory holds constant only to within the continuity
tolerance (exactly zero when they are exactly constant, the common case).
Then, for every pair,

- if $\varphi_p(q(t)) > \delta + r + \sigma_p + 2\tau_p + \varepsilon$ for
  every $t$, the pair is certified, so a trajectory that clears the margin by
  that much everywhere returns `kCertifiedFree`;
- if $\varphi_p(q(t)) < \delta - (r + \sigma_p + 2\tau_p)$ for some $t$, the
  check returns `kViolationFound`;
- `kInconclusive` is therefore possible only when some pair's clearance comes
  within that band of the margin, and its `Finding` then names an
  on-trajectory configuration whose reported distance lies in
  $[\delta - \tau_p,\ \delta + \tau_p + \varepsilon + \sigma_p + r]$.

Resolutions below what double precision can represent along a segment are
capped by a floating-point backstop.

## 7. Two corollaries

**It survives obstacle-surface perturbation below $\delta$.** A certificate
says every pair clears by more than $\delta$ at every time.
Moving any geometry's surface by less than $\delta$ therefore leaves the
trajectory collision-free.

**It is a property of the path.** The certificate depends only on the image of
$q$, not on its timing.
Retiming the trajectory afterwards, for example by time-optimal
parametrization, does not invalidate it.

## 8. What the proof does not cover

- **Tangency costs effort linear, not logarithmic, in $1/r$.** Where the
  trajectory's true clearance sits within oracle tolerance of the margin over
  a set of positive width, every node inside it is refined to the floor.
  Loosen the floor, filter intentional-contact pairs, or lower the margin.
- **Floating point.** The certificate is mathematical up to $\tau_p$ and
  $\varepsilon$. That is weaker than an interval-arithmetic proof.
- **A mesh means its convex hull.** Conservative in the certification
  direction; hollow or concave meshes can produce false violations.
- **Excluded models.** A trajectory that moves a coordinate of any joint
  other than revolute, prismatic, planar or screw raises (a quaternion
  coefficient is not an angle, so (4) gives it no bound). A negative margin,
  deformables, kinematic loops and reversed joints raise. Half spaces are
  admitted under these rules:
  - a pair of two half spaces raises;
  - a half space may be the static partner of a rotating body, such as an
    anchored ground plane under an arm, because $\lambda$ then bounds the
    arm's points and distance is symmetric;
  - a half space on the distal side of a revolute, planar or screw joint,
    relative to an unfiltered partner, raises at construction;
  - a half space on the distal side of a rotational coordinate of another
    joint type raises unless that coordinate is exactly constant along the
    trajectory.

## 9. The parallel search, in the fork

The C++ checker in PR #24924 runs Algorithm 1 on $K$ worker threads against
one shared stack. A worker that splits a node pushes the right child to the
shared stack when it holds fewer nodes than there are live workers, and
otherwise keeps both children. A call starts serially and hires helper threads
only after 64 nodes.
Every node's decisions depend only on its own control points and the active
set it inherits.
So the verdict and the reported witness are identical at every thread count,
as is the node count on a run that certifies.
Where a violation prunes the search, the node count varies with timing.
One exception: on a degenerate segment whose start and end times coincide,
every node maps to the same time, so the reported configuration (not its time)
may differ.
[relation-to-drake.md](relation-to-drake.md) describes how this package
organizes the same search.
