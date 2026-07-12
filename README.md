# S-Curve with Pre-Braking

## Abstract
S-curve trajectory generation is executed during system state transitions, such as active disengagement or deceleration to a stop state[cite: 4]. During operation, failure cases were encountered where the algorithm became numerically unstable[cite: 4]. Investigation through recorded data replay in simulation revealed that instability occurred when the initial kinematic state—specifically the initial acceleration—approached a critical value that causes the trajectory duration equation to become singular[cite: 4]. To address this, a pre-braking phase was developed that brings the system's kinematic state back within safe bounds before the main S-curve is executed, eliminating the instability across all tested operating conditions[cite: 4]. The pre-braking phase is a modified version of the Ruckig algorithm, adapted to the specific kinematic constraints of the system[cite: 4].

---

## 1. Implementation Summary

### 1.1 Algorithm Analysis
Customers reported invalid trajectory computations when the initial kinematic state falls outside the expected bounds[cite: 4]. In the legacy S-curve implementation, the duration of the whole trajectory, $T_{min}$, is pre-computed[cite: 4]. It was defined by the following expression[cite: 4]:

$$T_{min}=\frac{1}{6a_{max}(2a_{max}(n-1)+a_{0})^{2}v_{max}}[-24a_{max}^{3}(n-1)^{2}(p_{0}-p_{e})+12a_{max}^{2}(n-1)(n(v_{0}^{2}-2v_{0}v_{max}+2v_{max}^{2})-2a_{0}(p_{0}-p_{e}))-2a_{0}a_{max}(3a_{0}(p_{0}-p_{e})-2(v_{0}^{2}-2v_{0}v_{max}+(3n+1)v_{max}^{2}))+3a_{0}^{2}v_{max}^{2}\frac{n}{n-1}]$$

Mathematically, the validity of this equation requires a non-zero denominator[cite: 4]. A singularity occurs when[cite: 4]:

$$a_{0}=-2a_{max}(n-1)$$

Using the parameters $v_{max}=0.1$, $a_{max}=0.2$, and $n=6$, the critical value is $a_{0}=-2(0.2)(5)=-2.0$[cite: 4]. As $a_{0}$ approaches this value, $T_{min}$ tends toward infinity, leading to numerical instability[cite: 4].

### 1.2 Pre-Braking Phase
Analysis shows that the location of the mathematical singularity is not static; rather, it varies as a function of the kinematic constraints $v_{max}$, $a_{max}$, and the boundary positions $p_{0}$ and $p_{e}$[cite: 4]. Because this instability shifts depending on the motion profile, a hard-coded safety margin is insufficient[cite: 4]. This necessitates a robust architectural change: the integration of a Pre-Braking Phase inspired by the Ruckig algorithm[cite: 4]. 

Unlike the legacy approach, which attempts to solve the entire motion with a single unstable equation, the pre-braking phase decouples the problem into two distinct steps[cite: 4]:
1. **State Correction:** Preliminary maneuvers bring the system's velocity or acceleration back within defined limits[cite: 4].
2. **Main Trajectory:** Once the state is guaranteed to be within a safe, non-singular region, the main S-curve is generated[cite: 4].

---

## 2. Modified Ruckig Pre-Braking Profile
The primary purpose of the pre-braking phase is to calculate "braking" trajectories—preliminary maneuvers used to bring a system's state (velocity or acceleration) back within its defined limits before the main trajectory begins[cite: 4]. The algorithm uses a conditional branching structure to identify the "worst-offender" kinematic limit[cite: 4]. By swapping arguments and flipping the sign of $j_{Max}$, we use the same core logic to handle both positive and negative boundary violations[cite: 4].

### 2.1 High-Level Logic & Calling Structure

```matlab
if (a0 > aMax)
    [t0, t1, t2...] = acceleration_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);
elseif (a0 < aMin)
    [t0, t1, t2...] = acceleration_brake(v0, a0, vMin, vMax, aMin, aMax, jMax);
elseif (v0 > vMax && v_at_aiszero(...) > vMin) || (a0 > 0 && v_at_aiszero(...) > vMax)
    [t0, t1, t2...] = velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);
elseif (v0 < vMin && v_at_aiszero(...) < vMax) || (a0 < 0 && v_at_aiszero(...) < vMin)
    [t0, t1, t2...] = velocity_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);
```

**`acceleration_brake`**
This function is triggered when the initial acceleration $a_{0}$ is outside the allowed bounds $[a_{min}, a_{max}]$[cite: 4].
* It calculates the duration required to bring the acceleration back to $a_{max}$ or $a_{min}$[cite: 4].
* It performs a predictive check using `v_at_aiszero` to determine if ramping to zero acceleration will cause a velocity limit violation[cite: 4].
* If an overshoot is likely, it implements a Fast Recovery 3-segment profile (flipping $a_{0}$ to $-a_{0}$) or hands control to the velocity brake logic[cite: 4].
* Otherwise, it computes the simple duration needed to reach the acceleration limit[cite: 4].

**`velocity_brake`**
This is utilized when the velocity $v_{0}$ exceeds its limits or is on a trajectory that will inevitably lead to a violation[cite: 4].
* It calculates the time needed to apply maximum jerk to reduce velocity as quickly as possible[cite: 4].
* It accounts for two distinct phases: a ramping phase (varying acceleration) and a constant acceleration phase (coasting at $a_{min}$ or $a_{max}$) if the acceleration limit is reached before the velocity target is met[cite: 4].
* It includes a 1-segment shortcut if the velocity limit is reached before the acceleration ramp-down can be completed[cite: 4].

### 2.2 Implementation Strategies (The "Braking" Cases)
The algorithm selects one of three primary jerk-limited profiles based on the severity of the state[cite: 4]:

| Strategy | Jerk Sequence | Description |
| :--- | :--- | :--- |
| **Fast Recovery (3-Seg)** | $[-j_{max}, 0, j_{max}]$ | **The "Symmetric Flip":** Flips $a_{0} \rightarrow -a_{0}$ to cancel velocity[cite: 4]. |
| **Velocity Coast (2-Seg)** | $[-j_{max}, 0]$ | **Ramp & Constant:** Ramps to $a_{limit}$ and holds until $v_{target}$[cite: 4]. |
| **Immediate Exit (1-Seg)** | $[-j_{max}]$ | **Direct Ramp:** Used when $v_{limit}$ is hit during the ramp[cite: 4]. |

---

## 3. Summary
The transition to a S-Curve with pre-braking architecture addresses a critical flaw in the legacy S-curve implementation, where the pre-computed trajectory duration $T_{min}$ suffered from a mathematical singularity[cite: 4]. When the initial acceleration $a_{0}$ reached the critical value $a_{0} = -2a_{max}(n-1)$, the denominator of the duration equation approached zero, causing $T_{min}$ to tend toward infinity and inducing numerical instability[cite: 4].

By integrating a Pre-Braking Phase inspired by the Ruckig algorithm, the motion is now decoupled into two distinct stages: State Correction and Main Trajectory[cite: 4]. The preliminary maneuvers utilize a conditional branching structure to identify kinematic "worst-offenders" and apply specific jerk-limited strategies: Fast Recovery (3-Seg), Velocity Coast (2-Seg), or Immediate Exit (1-Seg)[cite: 4]. This architectural shift ensures that the system's velocity and acceleration are brought within safe limits before the main S-curve begins, effectively bypassing the regions of mathematical instability[cite: 4].
