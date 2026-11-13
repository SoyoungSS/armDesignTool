"""Studio panels for recorded task state and local web viewer lifetime."""
from collections import deque
import time

from mujoco.studio import messages, plugin, viewer_protocol
from mujoco.studio.dear_imgui import dear_imgui as imgui
from mujoco.studio.implot import implot


def viewer_plugins(contract, idle_seconds):
    class TaskPlots(plugin.ViewerPlugin):
        def __init__(self):
            self.joints = contract.get("live_plot_joints", contract["robot_joints"][:2])
            self.site = contract["end_effector_sites"][0]
            self.series = ()
            self.samples = deque(maxlen=400)
            self.next_sample = 0.
            self.run_name = ""
            self.cycle = None

        def reset_series(self, session):
            self.samples.clear()
            self.series = ()
            self.next_sample = 0.
            self.cycle = None
            self.run_name = session.run.name
            self.joint_ids = [session.m.joint(name).dofadr[0] for name in self.joints]
            self.site_id = session.m.site(self.site).id

        def sample(self, data, cycle):
            if cycle != self.cycle:
                self.cycle = cycle
                self.samples.clear()
                self.next_sample = 0.
            if data.time + 1e-10 < self.next_sample:
                return
            self.next_sample = data.time + .05
            self.samples.append((float(data.time), *data.qfrc_actuator[self.joint_ids].tolist(),
                                 *data.site_xpos[self.site_id].tolist()))
            self.series = tuple(zip(*self.samples))

        @messages.handler
        def draw(self, _: messages.BuildGuiEvent):
            width = min(350, imgui.GetIO().DisplaySize.x - 24)
            imgui.SetNextWindowPos(imgui.Vec2(12, 72), imgui.Cond.FirstUseEver)
            imgui.SetNextWindowSize(imgui.Vec2(width, 355), imgui.Cond.FirstUseEver)
            opened = imgui.Begin("Live curves")
            try:
                if not opened:
                    return
                imgui.TextUnformatted(f"Run {self.run_name}  |  Round {self.cycle or 1}")
                series = self.series
                if len(series) < 2 or len(series[0]) < 2:
                    imgui.TextUnformatted("Press Play to collect data")
                    return
                groups = [
                    ("Joint effort", "N m / N", self.joints, series[1:1+len(self.joints)]),
                    ("TCP position", "m", ("X", "Y", "Z"), series[-3:]),
                ]
                for title, units, labels, signals in groups:
                    if implot.BeginPlot(title, imgui.Vec2(-1, 136)):
                        implot.SetupAxes("Simulation time (s)", units,
                                         int(implot.AxisFlags.AutoFit), int(implot.AxisFlags.AutoFit))
                        for label, signal in zip(labels, signals):
                            implot.PlotLine(label, series[0], signal)
                        implot.EndPlot()
            finally:
                imgui.End()

    class WebIdle(plugin.ViewerPlugin):
        def __init__(self):
            self.server = None
            self.last_client = time.monotonic()

        @messages.handler
        def initialized(self, event: viewer_protocol.ViewerInitEvent):
            if idle_seconds:
                self.server = getattr(event.viewer, "_web_server", None)

        def expired(self):
            if self.server is None:
                return False
            now = time.monotonic()
            if self.server.viewer_count:
                self.last_client = now
            return now - self.last_client >= idle_seconds

    return TaskPlots(), WebIdle()
