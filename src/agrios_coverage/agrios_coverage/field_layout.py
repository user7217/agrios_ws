import math 
from dataclasses import dataclass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np 
from shapely.geometry import Polygon 
from shapely.ops import unary_union

@dataclass
class VehicleParams:
    coverage_width: float = 1.575   # 7 rows x 22.5 cm
    overlap: float = 0.10
    headland: float = 6.0           # placeholder until turn geometry finds the minimum
    length: float = 1.8
    width: float = 1.5
    turn_radius: float = 2.14       # front Ackermann

    @property
    def effective_width(self):
        return self.coverage_width - self.overlap


CORNERS = [(-5.0, -5.0), (55.0, -5.0), (55.0, 35.0), (-5.0, 35.0)]


def signed_area(pts):
	a = 0.0 
	n = len(pts)
	for i in range(n):
		x1, y1 = pts[i]
		x2, y2, = pts[(i+1)%n]
		a += x1*y2 - x2*y1
	return a/2.0

def build_field(corners):
    pts = [(float(x), float(y)) for x, y in corners]
    if len(pts) != 4:
        raise ValueError(f"expected 4 corners, got {len(pts)}")

    area = signed_area(pts)
    if abs(area) < 1.0:
        raise ValueError(f"degenerate polygon, area {area:.3f} m^2")
    if area < 0:
        pts = pts[::-1]  # force counter-clockwise

    poly = Polygon(pts)
    if not poly.is_valid:
        raise ValueError("self-intersecting polygon, check corner order")
    return pts, poly


#field frame 
@dataclass
class FieldFrame:
    origin: np.ndarray	#origin of the field 
    u: np.ndarray       # unit vector along passes
    n: np.ndarray       # unit vector across passes, into the field
    length: float
    width: float

    def to_map(self, x, y): #local grid coordinates to global map coordinates 
        return self.origin + x * self.u + y * self.n

    def to_field(self, p): #global map coordinates to local grid coordinates 
        d = np.asarray(p, dtype=float) - self.origin
        return float(d @ self.u), float(d @ self.n)


def field_frame(pts, poly, start_hint=(0.0, 0.0), tol=1e-6): #constructing teh coordinate system 
    count = len(pts)
    edges = []
    for i in range(count): #find polygon edges 
        a = np.array(pts[i])
        b = np.array(pts[(i + 1) % count])
        edges.append((i, b - a))

    max_len = max(np.linalg.norm(e) for _, e in edges)
    candidates = [(i, e) for i, e in edges if np.linalg.norm(e) > max_len - tol]
    hint = np.asarray(start_hint, dtype=float)
    i, e = min(candidates, key=lambda ie: np.linalg.norm(np.array(pts[ie[0]]) - hint))
 
    #calculate direction vectors
    length = float(np.linalg.norm(e)) #normalize longest edge
    u = e / length 
    n = np.array([-u[1], u[0]])  # u rotated +90 deg; interior side for a CCW polygon
    #origin 
    origin = np.array(pts[i])

    #calculate width 
    ys = [float((np.array(p) - origin) @ n) for p in pts]
    #check polygon lies on left side of base edge 
    if min(ys) < -1e-6:
        raise ValueError("interior not on the left of the base edge, polygon not CCW")
    width = max(ys)

    #validate rectangular 
    if abs(poly.area - length * width) > 1e-3 * poly.area:
        raise ValueError("field is not a rectangle")

    return FieldFrame(origin, u, n, length, width)


@dataclass
class Pass:
    index: int
    start: np.ndarray   # map frame
    end: np.ndarray
    heading: float      # rad, map frame
    direction: int      # +1 along u, -1 against u
    y: float            # field-frame offset across the field


@dataclass
class Transition:
    from_pass: int
    to_pass: int
    start_pose: tuple   # (x, y, heading), map frame
    end_pose: tuple
    offset: float       # lateral distance between the two passes
    turn_left: bool
    at_far_end: bool    # True if the turn is at x = length
    needs_reverse: bool # offset < 2R, so no forward U-turn fits


def pass_offsets(width, params):
    w_cov = params.coverage_width
    span = width - w_cov
    if span <= 0:
        return [width / 2.0]
    n = math.ceil(span / params.effective_width - 1e-9) + 1
    spacing = span / (n - 1)
    return [w_cov / 2.0 + k * spacing for k in range(n)]


def build_passes(frame, params):
    if params.width > params.coverage_width:
        raise ValueError("chassis wider than coverage strip, first pass overhangs the edge")

    x0 = params.headland
    x1 = frame.length - params.headland
    if x1 <= x0:
        raise ValueError("headland too deep for this field length")

    passes = []
    for k, y in enumerate(pass_offsets(frame.width, params)):
        d = 1 if k % 2 == 0 else -1
        xs, xe = (x0, x1) if d == 1 else (x1, x0)
        hv = frame.u * d
        passes.append(Pass(
            index=k,
            start=frame.to_map(xs, y),
            end=frame.to_map(xe, y),
            heading=math.atan2(hv[1], hv[0]),
            direction=d,
            y=y,
        ))
    return passes



def build_transitions(passes, params):
    out = []
    for a, b in zip(passes, passes[1:]):
        offset = b.y - a.y
        out.append(Transition(
            from_pass=a.index,
            to_pass=b.index,
            start_pose=(float(a.end[0]), float(a.end[1]), a.heading),
            end_pose=(float(b.start[0]), float(b.start[1]), b.heading),
            offset=offset,
            turn_left=(a.direction == 1),
            at_far_end=(a.direction == 1),
            needs_reverse=offset < 2.0 * params.turn_radius,
        ))
    return out


def field_rect(frame, x0, x1, y0, y1):
    corners = [frame.to_map(x0, y0), frame.to_map(x1, y0),
               frame.to_map(x1, y1), frame.to_map(x0, y1)]
    return Polygon([tuple(c) for c in corners])


def coverage_strips(frame, passes, params):
    half = params.coverage_width / 2.0
    x0, x1 = params.headland, frame.length - params.headland
    return [field_rect(frame, x0, x1, p.y - half, p.y + half) for p in passes]


def coverage_stats(frame, poly, passes, params):
    strips = coverage_strips(frame, passes, params)
    union = unary_union(strips)
    interior = field_rect(frame, params.headland, frame.length - params.headland,
                          0.0, frame.width)
    return {
        "field_area": poly.area,
        "interior_area": interior.area,
        "headland_area": poly.area - interior.area,
        "interior_covered": union.intersection(interior).area / interior.area,
        "outside_field": union.difference(poly).area,
        "overlap_area": sum(s.area for s in strips) - union.area,
        "strips": strips,
    }


def plot_layout(frame, poly, passes, stats, params, path):
    fig, ax = plt.subplots(figsize=(11, 8))

    fx, fy = poly.exterior.xy
    ax.plot(fx, fy, "k-", lw=2, label="field")

    for hl in (field_rect(frame, 0.0, params.headland, 0.0, frame.width),
               field_rect(frame, frame.length - params.headland, frame.length,
                          0.0, frame.width)):
        hx, hy = hl.exterior.xy
        ax.fill(hx, hy, color="orange", alpha=0.25)

    for s in stats["strips"]:
        sx, sy = s.exterior.xy
        ax.fill(sx, sy, color="tab:green", alpha=0.15, lw=0)

    for p in passes:
        d = p.end - p.start
        ax.arrow(p.start[0], p.start[1], d[0], d[1], length_includes_head=True,
                 head_width=0.6, head_length=1.2, color="tab:green", lw=0.8)

    ax.plot(*passes[0].start, "bo", ms=8, label="pass 0 start")
    ax.plot(0.0, 0.0, "r^", ms=10, label="robot spawn")

    ax.set_aspect("equal")
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.set_title(f"{len(passes)} passes, spacing {passes[1].y - passes[0].y:.3f} m")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)	


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def rotate(points, deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def report(label, corners, params, png=None):
    pts, poly = build_field(corners)
    f = field_frame(pts, poly)
    passes = build_passes(f, params)
    trans = build_transitions(passes, params)
    spacing = passes[1].y - passes[0].y if len(passes) > 1 else 0.0

    print(f"--- {label}")
    print(f"origin {np.round(f.origin, 3)}  u {np.round(f.u, 3)}")
    print(f"passes {len(passes)}  spacing {spacing:.4f} m  "
          f"realised overlap {params.coverage_width - spacing:.4f} m")
    print(f"pass length {np.linalg.norm(passes[0].end - passes[0].start):.2f} m")
    for p in passes[:3]:
        print(f"  pass {p.index:2d}  dir {p.direction:+d}  y {p.y:7.4f}  "
              f"start {np.round(p.start, 2)}  end {np.round(p.end, 2)}  "
              f"hdg {math.degrees(p.heading):7.2f}")
    rev = sum(t.needs_reverse for t in trans)
    print(f"transitions {len(trans)}  needing reverse {rev}  "
          f"(2R = {2 * params.turn_radius:.2f} m)")
    for t in trans[:2]:
        side = "left" if t.turn_left else "right"
        end = "far" if t.at_far_end else "near"
        print(f"  {t.from_pass} -> {t.to_pass}  offset {t.offset:.4f} m  turn {side} at {end} end")
        st = coverage_stats(f, poly, passes, params)
    print(f"field {st['field_area']:.1f} m^2  interior {st['interior_area']:.1f}  "
          f"headland {st['headland_area']:.1f}")
    print(f"interior covered {100 * st['interior_covered']:.3f} %  "
          f"outside field {st['outside_field']:.4f} m^2  overlap {st['overlap_area']:.1f} m^2")
    if png:
        plot_layout(f, poly, passes, st, params, png)
        print(f"wrote {png}")


if __name__ == "__main__":
    report("axis-aligned", CORNERS, VehicleParams(), "layout_axis.png")
    report("rotated 30 deg", rotate(CORNERS, 30), VehicleParams(), "layout_rot30.png")
    report("overlap 0.097", CORNERS, VehicleParams(overlap=0.097), "layout_ov097.png")

    print("--- bowtie")
    try:
        build_field([CORNERS[0], CORNERS[2], CORNERS[1], CORNERS[3]])
        print("NOT rejected -- bug")
    except ValueError as err:
        print(f"rejected: {err}")

