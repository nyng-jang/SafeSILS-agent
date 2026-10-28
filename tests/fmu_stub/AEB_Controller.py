"""Stand-in for the Simulink FMU, used to test safesils.external.FMUController without MATLAB.

Built with pythonfmu; it exposes the same port names as matlab/build_aeb_model.m
and runs the Python reference controller on the decoded raw CAN slots.
"""
from pythonfmu import Fmi2Causality, Fmi2Slave, Real

from safesils.bus import Message, decode
from safesils.controller import AEBController
from safesils.external import FRAME_FIELDS, FRAME_SLOTS, INPUT_PORTS, OUTPUT_NAMES, STATES


class AEB_Controller(Fmi2Slave):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.u = {}
        for name, width in INPUT_PORTS:
            names = [name] if width == 1 else [f"{name}[{i}]" for i in range(1, width + 1)]
            for n in names:
                self.u[n] = 0.0
                self.register_variable(Real(n, causality=Fmi2Causality.input,
                                            getter=lambda n=n: self.u[n], setter=lambda v, n=n: self.u.__setitem__(n, v)))
        self.y = {n: 0.0 for n in OUTPUT_NAMES}
        for n in OUTPUT_NAMES:
            self.register_variable(Real(n, causality=Fmi2Causality.output, getter=lambda n=n: self.y[n]))
        self.ctrl = AEBController()

    def _msgs(self, port):
        width = len(FRAME_FIELDS)
        out = []
        for k in range(FRAME_SLOTS):
            f = [self.u[f"{port}[{k * width + j + 1}]"] for j in range(width)]
            if f[0]:
                out.append(Message(port, int(f[1]), decode(int(f[3]), int(f[4]), int(f[5]), int(f[6])), int(f[2]), 0, 0))
        return out

    def do_step(self, current_time, step_size):
        if self.u["reset"]:
            self.ctrl = AEBController()
        out = self.ctrl.step(int(self.u["t_ms"]), self._msgs("radar_frames"), self._msgs("camera_frames"),
                             self.u["ego_speed_mps"], bool(self.u["driver_brake"]), self.u["steer_rate_dps"],
                             self.u["fault_value"] if self.u["fault_active"] else None)
        for n in OUTPUT_NAMES:
            v = out[n]
            self.y[n] = float(STATES.index(v)) if n == "aeb_state" else float(v)
        return True
