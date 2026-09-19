"""Progress bars and ETA for the long training runs.

Both trainers are unattended multi-hour jobs whose only feedback was one line per
iteration. That is enough to read afterwards and not enough to answer the question you
actually have at minute forty -- *is this working, and when will it finish?*

Three constraints shaped this, and they are why there is no `tqdm`:

* **No new dependency.** `uv.lock` is shared with the Mac, and a re-lock to add a
  progress bar would put the two machines on different resolutions for no gain. Stdlib
  only.
* **It must survive redirection.** These runs are launched with the output going to a
  log file. A `\\r`-driven bar written to a file produces one enormous unreadable line,
  so when stdout is not a TTY the bar degrades to a periodic newline-terminated line.
* **It must never be load-bearing.** Nothing here touches the RNG, the archive or the
  scorecard. It is pure stderr/stdout decoration and cannot affect a result.
"""

from __future__ import annotations

import shutil
import sys
import time
from collections import deque

#: Redraw at most this often on a TTY. Faster than this is invisible to a human and
#: costs syscalls in the hot path between evaluations.
MIN_REDRAW_S = 0.25

#: In a log file, one line per this many seconds. Sparse enough that a ten-hour run does
#: not produce a hundred thousand lines, frequent enough to prove liveness.
LOG_INTERVAL_S = 60.0

_FULL = "█"     # full block
_PART = "░"     # light shade, for the unfilled remainder


def fmt_duration(seconds: float) -> str:
    """`72s` / `12.4m` / `1h 23m`. Short enough to sit inside a status line."""
    if seconds < 0 or seconds != seconds:          # negative or NaN
        return "--"
    if seconds < 90:
        return f"{seconds:.0f}s"
    if seconds < 3600:
        return f"{seconds / 60:.1f}m"
    h, rem = divmod(int(seconds), 3600)
    return f"{h}h {rem // 60:02d}m"


class Progress:
    """A determinate progress bar with a linear ETA.

    The ETA is a mean rate extrapolated to `total`, not a windowed one. For these
    trainers the per-unit cost is near-uniform -- every evaluation is the same episode
    length on the same maps -- so the mean is both the lower-variance estimator and the
    one that stops jumping around when a single worker straggles.

    **The rate is measured over a trailing window, not the whole run**, and both
    corrections were forced by measurement rather than taste:

    * `spawn` costs ~20 s to bring up 15 workers. Folding that into the per-unit rate
      made the first estimate of a 2.5-minute run read *22.8 minutes*. So the clock
      starts at the first completion, and until there are two the ETA is `--`.
    * A run is not homogeneous. The seeding phase puts 6 genomes on 15 workers and so
      has a far worse per-evaluation wall rate than the saturated steady state; averaged
      over the whole run that transient still read *2.1m remaining* when 1.1m was left.
      A window ages it out instead of carrying it to the end.
    """

    #: Completions kept for the rate estimate. Long enough to smooth the jitter of
    #: workers finishing in bursts, short enough to forget a phase change within one
    #: iteration's batch.
    WINDOW = 64

    def __init__(self, total: int, *, label: str = "", stream=None,
                 width: int | None = None, enabled: bool = True) -> None:
        self.total = max(int(total), 1)
        self.label = label
        self.stream = stream if stream is not None else sys.stdout
        self.enabled = enabled
        self.done = 0
        self.t0 = time.perf_counter()
        #: Trailing (wall time, completed) samples, oldest first. The first entry is
        #: appended at the first completion, so pool startup never enters the rate.
        self._samples: deque[tuple[float, int]] = deque(maxlen=self.WINDOW)
        self._last_draw = 0.0
        self._painted = False
        # A file gets periodic lines; a terminal gets a repainted bar.
        self.tty = bool(enabled and getattr(self.stream, "isatty", lambda: False)())
        if width is not None:
            self.width = width
        else:
            cols = shutil.get_terminal_size((100, 24)).columns if self.tty else 100
            # Leave room for the label, counters, elapsed and ETA around the bar.
            self.width = max(10, min(32, cols - 62))

    # -- rendering ---------------------------------------------------------------

    def _bar(self, frac: float) -> str:
        filled = int(round(frac * self.width))
        return _FULL * filled + _PART * (self.width - filled)

    def eta_seconds(self) -> float:
        """Seconds remaining, or NaN while there is not yet an honest estimate.

        Needs two completions, not one: with a single sample the only rate available
        still includes whatever one-off cost preceded it.
        """
        if len(self._samples) < 2:
            return float("nan")
        (t_old, d_old), (t_new, d_new) = self._samples[0], self._samples[-1]
        advanced = d_new - d_old
        if advanced <= 0:
            return float("nan")
        rate = (t_new - t_old) / advanced
        return rate * max(self.total - self.done, 0)

    def render(self, suffix: str = "") -> str:
        frac = min(self.done / self.total, 1.0)
        elapsed = time.perf_counter() - self.t0
        eta = self.eta_seconds()
        parts = [
            f"{self.label}" if self.label else "",
            f"[{self._bar(frac)}]",
            f"{frac * 100:5.1f}%",
            f"{self.done}/{self.total}",
            f"elapsed {fmt_duration(elapsed)}",
            f"ETA {fmt_duration(eta)}",
        ]
        if suffix:
            parts.append(suffix)
        return "  ".join(p for p in parts if p)

    # -- driving -----------------------------------------------------------------

    def update(self, done: int | None = None, *, step: int = 0, suffix: str = "",
               force: bool = False) -> None:
        """Advance to `done` (or by `step`) and repaint if it is time to."""
        if done is not None:
            self.done = int(done)
        elif step:
            self.done += step
        # Record progress the first time anything completes, and on every advance
        # after: startup lands before the first sample rather than inside the rate.
        if self.done > 0 and (not self._samples or self.done != self._samples[-1][1]):
            self._samples.append((time.perf_counter(), self.done))
        if not self.enabled:
            return

        now = time.perf_counter()
        interval = MIN_REDRAW_S if self.tty else LOG_INTERVAL_S
        if not force and (now - self._last_draw) < interval:
            return
        self._last_draw = now

        line = self.render(suffix)
        if self.tty:
            # \r + erase-to-end-of-line, so a shorter line cannot leave debris behind.
            self.stream.write("\r\x1b[K  " + line)
        else:
            self.stream.write("  " + line + "\n")
        self.stream.flush()
        self._painted = True

    def log(self, line: str) -> None:
        """Print a permanent line without the bar overwriting it, or it the bar.

        The per-iteration summaries are the record of the run and must survive in
        scrollback; the bar is transient. On a TTY that means erasing the bar, writing
        the line, then repainting the bar underneath it.
        """
        if self.tty and self._painted:
            self.stream.write("\r\x1b[K")
        self.stream.write(line + "\n")
        self.stream.flush()
        self._painted = False
        if self.enabled and self.tty:
            self.update(force=True)

    def close(self, suffix: str = "") -> None:
        """Finish the bar and leave the cursor on a fresh line.

        Called on the way out of the `with` block *including* on exception, so a run
        interrupted with Ctrl-C does not leave the shell prompt painted over a bar.
        """
        if not self.enabled:
            return
        self.update(force=True, suffix=suffix)
        if self._painted and self.tty:
            self.stream.write("\n")
            self.stream.flush()

    def __enter__(self) -> Progress:
        return self

    def __exit__(self, *exc) -> None:
        self.close()


__all__ = ["Progress", "fmt_duration"]
