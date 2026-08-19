"""Cell: one MuJoCo-simulated station running cycles against a target.

Phase 2 scope: a single cell, one cycle at a time. Readiness checks run
before every cycle (DESIGN.md section 3), in the order the design lays
them out — config resolution, calibration lookahead, sensors, capability
— and any failure there is a Refusal, not an exception.
"""
import time
import uuid
from typing import FrozenSet, Optional

import mujoco
import numpy as np

from runtime import config as config_module
from runtime import hardware_limits
from runtime import variants as variants_module
from runtime.controller import ProportionalController, actuated_qpos, actuated_qvel
from runtime.records import CycleRecord, write_cycle_record


class Cell:
    def __init__(
        self,
        cell_id: str,
        mjcf_path: str,
        declared_sensors: FrozenSet[str] = frozenset(),
        capable_variants: FrozenSet[str] = frozenset({"default"}),
        robot_name: Optional[str] = None,
    ):
        self.cell_id = cell_id
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        # Hardware (DESIGN.md section 1: "static-ish", not resolved
        # through Config) — what sensors this cell physically has, and
        # which variants it's capable of (reach/grasp).
        self.declared_sensors = declared_sensors
        self.capable_variants = capable_variants

        # robot_name keys hardware_limits.MAX_JOINT_VELOCITY_RAD_S (no
        # velocity limit is encoded in the MJCF itself — see
        # hardware_limits.py). None means "unknown robot" and the
        # mid-cycle velocity check is skipped rather than guessed at.
        self.robot_name = robot_name

        config = self._resolve_config()
        self.controller = ProportionalController(kp=config["control_gain_kp"]["value"])

        # Calibration is a value + age (DESIGN.md section 1), enforced
        # via the pre-cycle lookahead check in run_cycle.
        self.calibration_value = 0.0
        self._calibrated_at = time.monotonic()

        # Obstruction / halt state (DESIGN.md section 5, "Failure — task
        # outcome (obstructing)"). The sim has no part-drop physics to
        # detect this from, so it's injected directly rather than derived
        # — consistent with the project's "every failure is one you
        # inject yourself" framing.
        self.halted = False
        self.halt_reason: Optional[str] = None

        # done (DESIGN.md section 3): false at the start of every cycle,
        # true once it completes within the takt budget. A cell can't
        # judge takt in isolation — this is the signal the Line's clock
        # is meant to read at each tick boundary to decide over-takt.
        # No Line exists yet to read it that way; run_cycle still decides
        # over-takt internally too (see TODO.md), so treat this as real
        # state now, not yet the sole source of truth.
        self.done = False

    def _resolve_config(self):
        return config_module.resolve(self.cell_id)

    def calibration_age_s(self) -> float:
        return time.monotonic() - self._calibrated_at

    def obstruct(self, reason: str = "obstruction") -> None:
        """Inject an obstructing failure: a part dropped/misplaced inside
        this cell's workspace, physically blocking the next cycle.

        If the resolved config's autoclear is on, this is a no-op on
        cell state (DESIGN.md section 1c: the failure would still be
        logged by whatever calls this, only the *response* changes) —
        the cell never halts. Otherwise the cell halts and every
        run_cycle call is refused until clear_failure() is called.
        """
        autoclear = self._resolve_config().get("autoclear", {"value": False})["value"]
        if autoclear:
            return
        self.halted = True
        self.halt_reason = reason

    def clear_failure(self) -> None:
        """Manual clear (DESIGN.md section 1c) — an operator explicitly
        clearing an obstruction (e.g. a `clear_failure {cell_id}` command
        on the commands/ channel; not yet wired to that channel in this
        build, see TODO.md)."""
        self.halted = False
        self.halt_reason = None

    def _record(
        self,
        outcome: str,
        reason: Optional[str],
        config: dict,
        part_id: Optional[str],
        variant: Optional[str],
        takt_s: float,
        sim_steps: int,
        duration_s: float,
    ) -> CycleRecord:
        record = CycleRecord(
            cell_id=self.cell_id,
            cycle_id=str(uuid.uuid4()),
            part_id=part_id,
            variant=variant,
            config=config,
            calibration_value=self.calibration_value,
            calibration_age_s=self.calibration_age_s(),
            outcome=outcome,
            reason=reason,
            duration_s=duration_s,
            takt_s=takt_s,
            sim_steps=sim_steps,
        )
        write_cycle_record(record)
        return record

    def _refuse(
        self,
        reason: str,
        config: dict,
        part_id: Optional[str],
        variant: Optional[str],
        takt_s: float = 0.0,
    ) -> CycleRecord:
        """Build, write, and return a refusal record — a decision made
        before attempting a cycle, not an exception (DESIGN.md section 5:
        "Refusals are decisions a cell makes before attempting a cycle —
        recorded outcomes, not exceptions.")."""
        return self._record("refusal", reason, config, part_id, variant, takt_s, sim_steps=0, duration_s=0.0)

    def run_cycle(
        self,
        target_qpos: np.ndarray,
        part_id: Optional[str] = None,
        variant: str = "default",
    ) -> CycleRecord:
        """Run one cycle: readiness checks first (DESIGN.md section 3, in
        order — config, calibration, sensors, capability), then step the
        controller toward target_qpos until the position error is within
        tolerance, or the takt-derived step budget runs out (over-takt
        failure).

        Any readiness check failing, or the cell being halted by an
        unresolved obstruction, produces a Refusal instead of an attempt.
        """
        self.done = False

        # 1. Config resolution can itself fail (malformed file, invalid
        # value). Caught broadly on purpose: DESIGN.md section 5 says any
        # such failure must become a Refusal, not a crash, regardless of
        # the specific cause.
        try:
            config = self._resolve_config()
        except Exception as exc:  # noqa: BLE001 - intentional, see above
            return self._refuse(f"config_unresolved: {exc}", config={}, part_id=part_id, variant=variant)

        takt_s = config["takt_s"]["value"]

        if self.halted:
            return self._refuse(self.halt_reason, config, part_id, variant, takt_s)

        # 2. Calibration lookahead: refuse if age + takt would cross the
        # expiry threshold before this cycle is expected to finish.
        calibration_max_age_s = config.get("calibration_max_age_s", {"value": float("inf")})["value"]
        if self.calibration_age_s() + takt_s >= calibration_max_age_s:
            return self._refuse("calibration_stale", config, part_id, variant, takt_s)

        # 3. Sensors required by the current variant must all be declared.
        try:
            required_sensors = variants_module.requirements(variant)["required_sensors"]
        except KeyError as exc:
            return self._refuse(f"capability_mismatch: {exc}", config, part_id, variant, takt_s)
        missing_sensors = required_sensors - self.declared_sensors
        if missing_sensors:
            return self._refuse(
                f"sensor_missing: {sorted(missing_sensors)}", config, part_id, variant, takt_s
            )

        # 4. The current variant must be within this cell's capability.
        if variant not in self.capable_variants:
            return self._refuse(
                f"capability_mismatch: variant {variant!r} not supported by this cell",
                config,
                part_id,
                variant,
                takt_s,
            )

        tolerance = config["position_tolerance_rad"]["value"]
        physics_dt = self.model.opt.timestep
        max_steps = max(1, int(takt_s / physics_dt))

        # Logic fault: the target itself asks for a joint position past
        # this hardware's true physical range (DESIGN.md section 1b: "an
        # out-of-range joint angle" is one of the adversarial instruction
        # shapes; section 5's failure table: "joint limits -> logic
        # fault"). Checked against the real limits read from the model
        # (hardware_limits.actuated_position_range), not a
        # site-configured softer limit — no such config field exists yet
        # (see TODO.md). Checked up front rather than discovered via
        # physics: MuJoCo would just physically clamp the joint at its
        # limit and let the cycle silently run out the clock as an
        # indistinguishable over-takt failure otherwise (verified by
        # hand before this was built).
        position_range = hardware_limits.actuated_position_range(self.model)
        out_of_range = np.where((target_qpos < position_range[:, 0]) | (target_qpos > position_range[:, 1]))[0]
        if out_of_range.size:
            return self._record(
                "failure",
                f"logic_fault: target out of joint range at actuator indices {out_of_range.tolist()}",
                config,
                part_id,
                variant,
                takt_s,
                sim_steps=0,
                duration_s=0.0,
            )

        # Safety violation: actual joint velocity exceeds this hardware's
        # datasheet limit mid-cycle (section 5: "velocity ... exceeds a
        # defined threshold" -> hard stop, cell pulled out immediately).
        # Only checked when robot_name is known — hardware_limits has no
        # entry to check against otherwise.
        max_velocity = hardware_limits.max_joint_velocity(self.robot_name) if self.robot_name else None

        reached = False
        steps_taken = 0
        for steps_taken in range(1, max_steps + 1):
            error = self.controller.step(self.model, self.data, target_qpos)
            mujoco.mj_step(self.model, self.data)

            if max_velocity is not None:
                qvel = actuated_qvel(self.model, self.data)
                if np.any(np.abs(qvel) > max_velocity):
                    self.halted = True
                    self.halt_reason = "safety_violation: joint velocity exceeded datasheet limit"
                    return self._record(
                        "failure",
                        self.halt_reason,
                        config,
                        part_id,
                        variant,
                        takt_s,
                        sim_steps=steps_taken,
                        duration_s=steps_taken * physics_dt,
                    )

            if np.max(np.abs(error)) < tolerance:
                reached = True
                break

        self.done = reached

        return self._record(
            "success" if reached else "failure",
            None if reached else "over_takt",
            config,
            part_id,
            variant,
            takt_s,
            sim_steps=steps_taken,
            duration_s=steps_taken * physics_dt,
        )

    def current_qpos(self) -> np.ndarray:
        return actuated_qpos(self.model, self.data)
