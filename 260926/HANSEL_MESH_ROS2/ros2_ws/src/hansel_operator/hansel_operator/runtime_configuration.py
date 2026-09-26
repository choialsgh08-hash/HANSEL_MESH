"""RQT configuration model. Driven from one UI thread; no ROS/Qt imports.

Endpoints return futures: read(names) -> dict, write(values) -> None or error.
Cross-node writes are compensated, not a distributed atomic transaction.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import re
import time

import yaml

MOTOR_NAMES = ('straight_rpm', 'turn_rpm', 'maximum_pwm_percent', 'minimum_pwm_percent')
HEAD_NAMES = ('head_min_angle_deg', 'head_center_angle_deg', 'head_max_angle_deg')


@dataclass(frozen=True)
class Topology:
    units: tuple
    roles: dict

    def __post_init__(self):
        if not self.units or len(set(self.units)) != len(self.units):
            raise ValueError('robot_chain ordered_units must be nonempty and unique')
        if any(not isinstance(u, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', u) for u in self.units):
            raise ValueError('robot_chain unit IDs must be valid ROS namespace components')
        if set(self.roles) != set(self.units) or any(r not in ('head', 'rear') for r in self.roles.values()):
            raise ValueError('robot_chain roles must define head/rear for every configured unit')
        if list(self.roles.values()).count('head') != 1 or self.roles[self.units[0]] != 'head':
            raise ValueError('robot_chain must have exactly one Head, first in ordered_units')

    @property
    def head(self):
        return next(u for u in self.units if self.roles[u] == 'head')

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        chain = data['robot_chain']
        return cls(tuple(chain['ordered_units']), dict(chain['roles']))


def validate_update(changes, values, head_only=False):
    allowed = set(HEAD_NAMES) if head_only else set(MOTOR_NAMES) - {'minimum_pwm_percent'}
    if not changes or not set(changes) <= allowed:
        raise ValueError('Unsupported parameter update')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in changes.values()):
        raise ValueError('Requested values must be finite numbers')
    effective = {**values, **changes}
    if head_only:
        low, center, high = (effective[n] for n in HEAD_NAMES)
        if not -180 <= low < center < high <= 180:
            raise ValueError('Head limits must satisfy -180 <= minimum < center < maximum <= 180')
    else:
        if effective['straight_rpm'] <= 0 or effective['turn_rpm'] <= 0:
            raise ValueError('RPM must be greater than zero')
        minimum, maximum = effective['minimum_pwm_percent'], effective['maximum_pwm_percent']
        if not 0 <= minimum <= maximum <= 100 or maximum <= 0:
            raise ValueError(f'Max PWM must be > 0, >= minimum_pwm_percent ({minimum:g}) and <= 100')


class RuntimeConfiguration:
    def __init__(self, topology, endpoints, *, clock=time.monotonic, timeout=3.0, write_guard=None):
        self.topology, self.endpoints = topology, endpoints
        self.clock, self.timeout = clock, timeout
        self.write_guard = write_guard or (lambda targets: "")
        self.active = None  # Never infer membership from service/network availability.
        self.chain_error = 'Waiting for ActiveChain'
        self.current = {}
        self.read_errors = {}
        self.last_result = ''
        self.stage = 'idle'
        self.revision = 0
        self._chain_revision = 0
        self._pending = {}
        self._batch_errors = {}
        self._late = {}
        self._remaining = set()
        self._snapshots = {}
        self._changes = {}
        self._sent = set()
        self._failure = ''

    @property
    def busy(self): return self.stage != 'idle'

    @property
    def recovery_required(self): return bool(self._remaining or self._late)

    def names(self, unit):
        return MOTOR_NAMES + (HEAD_NAMES if self.topology.roles[unit] == 'head' else ())

    def set_chain(self, ordered, active, relay):
        try:
            if tuple(ordered) != self.topology.units:
                raise ValueError('ActiveChain ordering differs from robot_chain.yaml; check installed configuration')
            if len(set(active)) != len(active) or not set(active) <= set(ordered):
                raise ValueError('ActiveChain contains duplicate/unknown active units')
            if not set(relay) <= set(ordered) or set(active) & set(relay):
                raise ValueError('ActiveChain relay/active membership is invalid')
            result = tuple(u for u in ordered if u in active)
            error = ''
        except ValueError as exc:
            result, error = None, str(exc)
        if result != self.active or error != self.chain_error:
            self.active, self.chain_error = result, error
            self._chain_revision += 1
            self.revision += 1
        return not error

    def common_value(self, name):
        if not self.active or any(u in self.read_errors or u not in self.current for u in self.active):
            return None
        values = [self.current[u][name] for u in self.active]
        return values[0] if all(v == values[0] for v in values) else None

    def _batch(self, stage, units, operation):
        self.stage = stage
        self._pending, self._batch_errors = {}, {}
        self._deadline = self.clock() + self.timeout
        for unit in units:
            try:
                if not self.endpoints[unit].ready():
                    raise RuntimeError('parameter services unavailable')
                self._pending[unit] = operation(unit)
            except Exception as exc:
                self._batch_errors[unit] = str(exc)

    def _read(self, stage, units):
        self._batch(stage, units, lambda u: self.endpoints[u].read(self.names(u)))

    def _take(self):
        if any(not f.done() for f in self._pending.values()) and self.clock() < self._deadline:
            return None
        values, errors, late = {}, dict(self._batch_errors), {}
        for unit, future in self._pending.items():
            if not future.done():
                errors[unit] = 'response timeout (request may still complete)'
                late[unit] = future
            else:
                try:
                    values[unit] = future.result()
                except Exception as exc:
                    errors[unit] = str(exc)
        return values, errors, late

    def _remember(self, values, errors):
        for unit, data in values.items():
            try:
                if set(data) != set(self.names(unit)) or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in data.values()
                ):
                    raise ValueError('node returned missing or non-finite numeric parameters')
                self.current[unit] = dict(data)
                self.read_errors.pop(unit, None)
            except Exception as exc:
                errors[unit] = str(exc)
        for unit, reason in errors.items():
            self.current.pop(unit, None)
            self.read_errors[unit] = reason
        self.revision += 1

    @staticmethod
    def _details(errors):
        return '; '.join(f'{u}: {reason}' for u, reason in errors.items())

    def refresh(self):
        if self.busy:
            return False
        self._read('refresh', self.topology.units)
        return True

    def apply(self, changes, *, head_only=False):
        if self.busy or self.recovery_required:
            self.last_result = 'Update blocked: another operation or unresolved rollback exists'
            return False
        if not self.active:
            self.last_result = self.chain_error or 'No active drive units'
            return False
        targets = tuple(u for u in self.active if not head_only or self.topology.roles[u] == 'head')
        if not targets:
            self.last_result = 'No active Head in ActiveChain'
            return False
        reason = self.write_guard(targets)
        if reason:
            self.last_result = "No changes sent: " + reason
            return False
        unavailable = [u for u in targets if not self.endpoints[u].ready()]
        if unavailable:
            self.last_result = 'No changes sent; unavailable parameter services: ' + ', '.join(unavailable)
            return False
        self._targets, self._head_only = targets, head_only
        self._changes, self._sent = dict(changes), set()
        self._snapshots = {}
        self._version = self._chain_revision
        self.last_result = 'Reading current values before applying to: ' + ', '.join(targets)
        self._read('snapshot', targets)
        return True

    def _write(self, unit):
        self._sent.add(unit)  # Include a rejected call: controller callbacks may have side effects.
        return self.endpoints[unit].write(self._changes)

    def _changed_chain(self):
        return self._version != self._chain_revision

    def _begin_rollback(self, reason, late=None):
        self._failure = reason
        self._late.update(late or {})
        self._remaining.update(self._sent)
        self.stage = 'idle'
        self.retry_rollback()

    def retry_rollback(self):
        if self.busy or not self.recovery_required:
            return False
        # Never issue a compensating write ahead of an unresolved original write.
        for unit, future in list(self._late.items()):
            if future.done():
                del self._late[unit]
        self._restore_targets = tuple(u for u in self._targets if u in self._remaining and u not in self._late)
        if not self._restore_targets:
            self._finish_rollback({})
            return False
        def restore(unit):
            return self.endpoints[unit].write({k:self._snapshots[unit][k] for k in self._changes})
        self._batch('rollback', self._restore_targets, restore)
        return True

    def _finish_rollback(self, errors):
        self.stage = 'idle'
        if self.recovery_required:
            detail = self._details(errors)
            self.last_result = (self._failure + ' | UNRESOLVED rollback: ' + ', '.join(sorted(self._remaining))
                                + (f' ({detail})' if detail else '')
                                + '. Further updates blocked; retry after responses/services recover.')
        else:
            self.last_result = self._failure + ' | rollback verified against the original snapshots'
        self.revision += 1

    def poll(self):
        if not self.busy:
            if any(f.done() for f in self._late.values()):
                self.retry_rollback()
            return
        result = self._take()
        if result is None:
            return
        values, errors, late = result
        stage = self.stage
        if stage in ('refresh', 'snapshot', 'verify', 'rollback_verify'):
            self._remember(values, errors)
            for future in late.values():
                discard = getattr(future, 'discard_read', None)
                if discard:
                    discard()
        if stage == 'refresh':
            self.stage = 'idle'
        elif stage == 'snapshot':
            if errors or self._changed_chain():
                self.last_result = 'No changes sent: ' + (self._details(errors) or 'ActiveChain changed during snapshot')
                self.stage = 'idle'
                return
            reason = self.write_guard(self._targets)
            if reason:
                self.last_result = "No changes sent: " + reason
                self.stage = "idle"
                return
            try:
                for unit in self._targets:
                    validate_update(self._changes, self.current[unit], self._head_only)
            except (ValueError, KeyError, TypeError) as exc:
                self.last_result = f'No changes sent: {unit}: {exc}'
                self.stage = 'idle'
                return
            self._snapshots = {u:dict(self.current[u]) for u in self._targets}
            # Recheck readiness for the entire group after snapshotting.
            if any(not self.endpoints[u].ready() for u in self._targets):
                self.last_result = 'No changes sent: service availability changed during snapshot'
                self.stage = 'idle'
                return
            self._batch('write', self._targets, self._write)
        elif stage == 'write':
            if errors or self._changed_chain():
                self._begin_rollback('Apply failed: ' + (self._details(errors) or 'ActiveChain changed'), late)
            else:
                self._read('verify', self._targets)
        elif stage == 'verify':
            for unit in self._targets:
                if unit not in errors and any(self.current[unit][k] != v for k,v in self._changes.items()):
                    errors[unit] = 'effective values differ from requested values'
            if errors or self._changed_chain():
                self._begin_rollback('Verification failed: ' + (self._details(errors) or 'ActiveChain changed'))
            else:
                self.stage = 'idle'
                self.last_result = 'Verified on every target: ' + ', '.join(self._targets)
                self.revision += 1
        elif stage == 'rollback':
            self._late.update(late)
            self._restore_errors = errors
            settled = [u for u in self._restore_targets if u not in late]
            self._read('rollback_verify', settled)
        elif stage == 'rollback_verify':
            errors.update(self._restore_errors)
            for unit in self._restore_targets:
                if unit in self._late or unit in errors:
                    continue
                if any(self.current[unit][k] != self._snapshots[unit][k] for k in self._changes):
                    errors[unit] = 'rollback readback differs from snapshot'
                else:
                    self._remaining.discard(unit)
            self._finish_rollback(errors)
