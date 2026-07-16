from pxr import Usd, UsdGeom, Gf
import aicps.tomato.wind.mesh_distance as mesh_distance

def find_stem_segments(pedicel_rig_data):
        """Returns just the stem segment children (segment_A/segement_A, 
        segment_B) - excludes tomato and calyx, unlike affected_parts as
        a whole."""
        segments = []
        for child in pedicel_rig_data.affected_parts:
            name = child.GetName().lower()
            if name.startswith("segement_a") or name.startswith("segment_a") or name.startswith("segment_b"):
                segments.append(child)
        return segments

def find_child_by_prefix(pedicel_rig_data, prefix):
    """Locates a specific part (tomato, calyx, segment) within a pedicel's
    direct children by name prefix, rather than treating the whole pedicel
    as one collision object."""
    prefix = prefix.lower()
    for child in pedicel_rig_data.affected_parts:
        if child.GetName().lower().startswith(prefix):
            return child
    return None

def visualize_fruit_pair(checker, pedicel_a, pedicel_b, stage, debug_root="/World/_DebugSpheres"):
    if stage.GetPrimAtPath(debug_root):
        stage.RemovePrim(debug_root)

    tomato_a = find_child_by_prefix(pedicel_a, "tomato")
    tomato_b = find_child_by_prefix(pedicel_b, "tomato")
    box_a, box_b = checker.world_bounds(tomato_a), checker.world_bounds(tomato_b)
    center_a = (box_a.GetMin() + box_a.GetMax()) / 2.0
    center_b = (box_b.GetMin() + box_b.GetMax()) / 2.0
    direction = (center_b - center_a).GetNormalized()

    radius_a = checker._ellipsoid_radius_along_direction(tomato_a, direction)
    radius_b = checker._ellipsoid_radius_along_direction(tomato_b, -direction)

    for name, center, radius in [("A", center_a, radius_a), ("B", center_b, radius_b)]:
        path = f"{debug_root}/{name}"
        sphere = UsdGeom.Sphere.Define(stage, path)
        sphere.GetRadiusAttr().Set(radius)
        UsdGeom.Xformable(sphere).AddTranslateOp().Set(center)
        sphere.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
        sphere.CreateDisplayOpacityAttr([0.35])




def clear_debug_spheres(stage, debug_root="/World/_DebugSpheres"):
    if stage.GetPrimAtPath(debug_root):
        stage.RemovePrim(debug_root)
 
class CollisionChecker:
    """
    Geometric (bounding-box) collision detection. Splits collision checks
    by part type rather than checking whole-pedicel boxes:
      - fruit vs fruit: strict, any overlap = reject
      - stem vs stem: expected near the crown, only reject above a
        volume-overlap threshold
      - trellis: not wired up yet, no trellis asset exists in the stage
        yet per the roadmap (Iteration 2)
    """

    def __init__(self, stage, stem_overlap_threshold=0.15, stem_notable_threshold=0.02,
             environment_prim=None, leaf_prims=None):
        self.stage = stage
        self.stem_overlap_threshold = stem_overlap_threshold
        self.stem_notable_threshold = stem_notable_threshold
        self.bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_],
        )

        self.environment_prim = environment_prim
        self._environment_points = None
        if environment_prim is not None:
            self._environment_points = mesh_distance.get_world_points(environment_prim)
            if self._environment_points is None:
                print(f"WARNING: no mesh points found under {environment_prim.GetPath()} - "
                    f"environment collision checks will be skipped")

        self.leaf_prims = leaf_prims
        self._leaf_point_sets = {}
        if leaf_prims:
            for leaf_prim in leaf_prims:
                pts = mesh_distance.get_world_points(leaf_prim)
                if pts is not None:
                    self._leaf_point_sets[str(leaf_prim.GetPath())] = pts
            if not self._leaf_point_sets:
                print(f"WARNING: no mesh points found under provided leaf_prims - "
                    f"leaf collision checks will be skipped")


    # shared helper for baselines and leaf collision checks
    def _leaf_part_distances(self, pedicel):
        """Returns {(leaf_path, part_type): distance} for every leaf this
        pedicel's tomato/stem segments can be measured against. part_type is
        'fruit' or 'stem'. No entry is created for a leaf/part pair that
        yields no valid distance - naturally covers pedicels with no nearby
        leaf at all."""
        result = {}
        tomato = find_child_by_prefix(pedicel, "tomato")
        segments = find_stem_segments(pedicel)

        for leaf_path, points in self._leaf_point_sets.items():
            if tomato is not None:
                d = mesh_distance.min_distance_from_cached_points(points, tomato)
                if d is not None:
                    result[(leaf_path, "fruit")] = d

            if segments:
                seg_dists = [
                    mesh_distance.min_distance_from_cached_points(points, seg)
                    for seg in segments
                ]
                seg_dists = [d for d in seg_dists if d is not None]
                if seg_dists:
                    result[(leaf_path, "stem")] = min(seg_dists)

        return result



    def world_bounds(self, prim):
        bound = self.bbox_cache.ComputeWorldBound(prim)
        return bound.ComputeAlignedBox()

    def boxes_overlap(self, box_a, box_b):
        min_a, max_a = box_a.GetMin(), box_a.GetMax()
        min_b, max_b = box_b.GetMin(), box_b.GetMax()
        return (
            min_a[0] <= max_b[0] and max_a[0] >= min_b[0] and
            min_a[1] <= max_b[1] and max_a[1] >= min_b[1] and
            min_a[2] <= max_b[2] and max_a[2] >= min_b[2]
        )

    def _volume(self, box):
        size = box.GetMax() - box.GetMin()
        return max(size[0], 0) * max(size[1], 0) * max(size[2], 0)

    def _intersection_volume(self, box_a, box_b):
        min_a, max_a = box_a.GetMin(), box_a.GetMax()
        min_b, max_b = box_b.GetMin(), box_b.GetMax()
        overlap = [
            max(0.0, min(max_a[i], max_b[i]) - max(min_a[i], min_b[i]))
            for i in range(3)
        ]
        return overlap[0] * overlap[1] * overlap[2]

    def check_pair(self, prim_a, prim_b):
        return self.boxes_overlap(self.world_bounds(prim_a), self.world_bounds(prim_b))

    def overlap_ratio(self, prim_a, prim_b):
        """Fraction of the SMALLER box's volume that's overlapping - a scale-
        independent measure so a tiny stem crossing another stem doesn't
        get judged by the same absolute-volume yardstick as two big ones."""
        box_a, box_b = self.world_bounds(prim_a), self.world_bounds(prim_b)
        if not self.boxes_overlap(box_a, box_b):
            return 0.0
        smaller_vol = min(self._volume(box_a), self._volume(box_b))
        if smaller_vol <= 0:
            return 0.0
        return self._intersection_volume(box_a, box_b) / smaller_vol
    
    def _bounding_sphere(self, prim):
        box = self.world_bounds(prim)
        center = (box.GetMin() + box.GetMax()) / 2.0
        size = box.GetMax() - box.GetMin()
        radius = (size[0] + size[1] + size[2]) / 6.0  # average half-extent
        return center, radius

    def _ellipsoid_radius_along_direction(self, prim, direction):
        """
        Approximates a tomato as an axis-aligned ellipsoid (using its actual
        bbox half-extents per axis) and returns its radius specifically in
        the given direction - not an isotropic average. A tomato that's thin
        toward its neighbor gets a small radius there, even if it's wide
        in some other direction.
        """
        box = self.world_bounds(prim)
        size = box.GetMax() - box.GetMin()
        a = max(size[0] / 2.0, 1e-6)
        b = max(size[1] / 2.0, 1e-6)
        c = max(size[2] / 2.0, 1e-6)

        dx, dy, dz = direction[0], direction[1], direction[2]
        denom = (dx / a) ** 2 + (dy / b) ** 2 + (dz / c) ** 2
        if denom <= 0:
            return max(a, b, c)
        return 1.0 / (denom ** 0.5)


    def check_environment_collision(self, pedicel, fruit_tolerance=0.009, stem_tolerance=0.009, debug=False):
        """
        Checks a pedicel's tomato AND stem segments against the cached
        static environment (trellis) points. No "natural contact" allowance
        here, unlike stem-vs-stem - the trellis is a rigid structure, so any
        real overlap should reject. Both tolerances start equal to the fruit
        tolerance as a placeholder - NOT yet calibrated. Use the same
        marker-visualization approach used for fruit/stem tolerances before
        trusting these numbers (see calibrate_environment_tolerance.py).

        Returns (rejected: bool, info: dict).
        """
        if self._environment_points is None:
            return False, {"skipped": "no environment configured"}

        tomato = find_child_by_prefix(pedicel, "tomato")
        worst = {"distance": float("inf"), "part": None}

        if tomato is not None:
            d = mesh_distance.min_distance_from_cached_points(self._environment_points, tomato)
            if d is not None and d < worst["distance"]:
                worst = {"distance": d, "part": tomato.GetName(), "tolerance": fruit_tolerance}

        for seg in find_stem_segments(pedicel):
            d = mesh_distance.min_distance_from_cached_points(self._environment_points, seg)
            if d is not None and d < worst["distance"]:
                worst = {"distance": d, "part": seg.GetName(), "tolerance": stem_tolerance}

        if worst["part"] is None:
            return False, {"skipped": "no tomato or stem parts found"}

        rejected = worst["distance"] < worst["tolerance"]
        if debug and rejected:
            print(f"  [TRELLIS REJECT] {pedicel.prim.GetName()} ({worst['part']})")
            print(f"    distance = {worst['distance']:.5f}  (tolerance = {worst['tolerance']})")

        return rejected, {"trellis_part": worst["part"], "trellis_distance": worst["distance"]}

    # fruit vs stem collisions checker
    def check_cross_stem_fruit_collision(self, pedicel_a, pedicel_b, contact_tolerance=0.009,
                                       relative_margin=0.001, debug=False):
        tomato_a = find_child_by_prefix(pedicel_a, "tomato")
        tomato_b = find_child_by_prefix(pedicel_b, "tomato")
        segments_a = find_stem_segments(pedicel_a)
        segments_b = find_stem_segments(pedicel_b)

        worst = {"distance": float("inf"), "pair": None}

        if tomato_b is not None:
            for seg in segments_a:
                d = mesh_distance.min_distance(seg, tomato_b)
                if d is not None and d < worst["distance"]:
                    worst = {"distance": d, "pair": f"{seg.GetName()} vs {tomato_b.GetName()}"}

        if tomato_a is not None:
            for seg in segments_b:
                d = mesh_distance.min_distance(tomato_a, seg)
                if d is not None and d < worst["distance"]:
                    worst = {"distance": d, "pair": f"{tomato_a.GetName()} vs {seg.GetName()}"}

        if worst["pair"] is None:
            return False, None

        baseline = getattr(self, "_baseline_cross_distance", {}).get(
            (pedicel_a.prim.GetName(), pedicel_b.prim.GetName())
        )

        if baseline is not None and baseline < contact_tolerance:
            rejected = worst["distance"] < baseline - relative_margin
        else:
            rejected = worst["distance"] < contact_tolerance

        if debug and rejected:
            print(f"  [CROSS STEM/FRUIT REJECT] {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}")
            print(f"    {worst['pair']}: distance = {worst['distance']:.5f}  baseline={baseline}")

        return rejected, worst["distance"]




    def capture_baselines(self, rig):
        """
        Records each pedicel's resting-pose distances - to the leaf cloud and
        to every other pedicel's fruit - before any rotation happens. Needed
        because some pairs are naturally close/touching in the source asset
        itself (02-04 fruit at 0.0027; every pedicel's own leaf, since leaves
        grow directly against the tomato/stem). An absolute tolerance would
        reject these forever, at any angle. Call this once per test run,
        right after rig.build(), before any controller/rotation exists.
        """
        self._baseline_fruit_distance = {}
        for i, a in enumerate(rig.pedicels):
            for b in rig.pedicels[i + 1:]:
                tomato_a = find_child_by_prefix(a, "tomato")
                tomato_b = find_child_by_prefix(b, "tomato")
                if tomato_a is None or tomato_b is None:
                    continue
                d = mesh_distance.min_distance(tomato_a, tomato_b)
                self._baseline_fruit_distance[frozenset([a.prim.GetName(), b.prim.GetName()])] = d

        # NEW: baseline for stem-vs-fruit across every pedicel pair, same idea
        # as fruit-vs-fruit above. Directional key (a,b) not frozenset, since
        # "a's stem vs b's fruit" and "b's stem vs a's fruit" are different
        # geometry and can have very different resting distances.
        self._baseline_cross_distance = {}
        for i, a in enumerate(rig.pedicels):
            for b in rig.pedicels[i + 1:]:
                _, d_ab = self.check_cross_stem_fruit_collision(a, b, contact_tolerance=-1)
                if d_ab is not None:
                    self._baseline_cross_distance[(a.prim.GetName(), b.prim.GetName())] = d_ab
                _, d_ba = self.check_cross_stem_fruit_collision(b, a, contact_tolerance=-1)
                if d_ba is not None:
                    self._baseline_cross_distance[(b.prim.GetName(), a.prim.GetName())] = d_ba

        # {(pedicel_name, leaf_path, part_type): distance}
        self._baseline_leaf_distance = {}
        if self._leaf_point_sets:
            for pedicel in rig.pedicels:
                for (leaf_path, part_type), d in self._leaf_part_distances(pedicel).items():
                    self._baseline_leaf_distance[(pedicel.prim.GetName(), leaf_path, part_type)] = d


    def check_leaf_collision(self, pedicel, fruit_tolerance=0.009, stem_tolerance=0.009,
                          relative_margin=0.001, debug=False):
        if not self._leaf_point_sets:
            return False, {"skipped": "no leaf points configured"}

        baselines = getattr(self, "_baseline_leaf_distance", {})
        current = self._leaf_part_distances(pedicel)

        if not current:
            return False, {"skipped": "no tomato or stem parts found near any leaf"}

        worst = None  # tracks the most severe rejection, if any
        closest = None  # tracks the closest pair overall, for info even if not rejected

        for (leaf_path, part_type), d in current.items():
            tolerance = fruit_tolerance if part_type == "fruit" else stem_tolerance
            baseline = baselines.get((pedicel.prim.GetName(), leaf_path, part_type))

            if baseline is not None and baseline < tolerance:
                rejected_here = d < baseline - relative_margin
            else:
                rejected_here = d < tolerance

            if closest is None or d < closest["distance"]:
                closest = {"distance": d, "part": part_type, "leaf": leaf_path, "baseline": baseline}

            if rejected_here and (worst is None or d < worst["distance"]):
                worst = {"distance": d, "part": part_type, "leaf": leaf_path, "baseline": baseline}

        rejected = worst is not None
        info = worst if rejected else closest
        info = {
            "leaf_part": info["part"],
            "leaf_path": info["leaf"],
            "leaf_distance": info["distance"],
            "baseline": info["baseline"],
        }

        if debug and rejected:
            print(f"  [LEAF REJECT] {pedicel.prim.GetName()} ({info['leaf_part']} vs {info['leaf_path'].split('/')[-6]})  "
                f"distance={info['leaf_distance']:.5f}  baseline={info['baseline']}")

        return rejected, info




    # replaced
    def check_fruit_collision(self, pedicel_a, pedicel_b, contact_tolerance=0.009, relative_margin=0.001, debug=False):
        tomato_a = find_child_by_prefix(pedicel_a, "tomato")
        tomato_b = find_child_by_prefix(pedicel_b, "tomato")
        if tomato_a is None or tomato_b is None:
            print(f"WARNING: no tomato found on {pedicel_a.prim.GetName()} or {pedicel_b.prim.GetName()}")
            return False

        distance = mesh_distance.min_distance(tomato_a, tomato_b)
        if distance is None:
            return False

        key = frozenset([pedicel_a.prim.GetName(), pedicel_b.prim.GetName()])
        baseline = getattr(self, "_baseline_fruit_distance", {}).get(key)

        if baseline is not None and baseline < contact_tolerance:
            # Naturally touching at rest - only reject if wind makes it WORSE.
            rejected = distance < baseline - relative_margin
        else:
            rejected = distance < contact_tolerance

        if debug and rejected:
            print(f"  [FRUIT REJECT] {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}  "
                f"distance={distance:.5f}  baseline={baseline}")
        return rejected

    # --- And thread debug through baseline_report so you can flip it on there ---

    def baseline_report(self, rig, debug=False):
        print("\n===== Baseline Collision Report (current pose) =====")
        any_rejected = False
        for i, pedicel_a in enumerate(rig.pedicels):
            for pedicel_b in rig.pedicels[i + 1:]:
                rejected, info = self.check_pedicel_pair(pedicel_a, pedicel_b, debug=debug)
                if rejected:
                    any_rejected = True
                    print(f"  REJECT: {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}  {info}")
                elif info["stem_min_distance"] is not None and info["stem_min_distance"] < self.stem_notable_threshold:
                    print(f"  ok (stem close but not rejected): {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}  distance={info['stem_min_distance']:.5f}")
        if not any_rejected:
            print("  No rejections.")
        print("======================================================\n")
        return any_rejected


    def check_stem_collision(self, pedicel_a, pedicel_b, stem_contact_tolerance=0.005, debug=False):
        """
        Mesh-distance stem check (replaces bbox volume-overlap ratio).
        Bounding boxes overestimate overlap for thin/curved segment geometry -
        confirmed visually on Pedicel_06<->07 (bbox ratio 0.427, but clearly
        not touching in viewport). Uses the same min_distance approach already
        validated for fruit.

        stem_contact_tolerance starts as a rough placeholder equal to the
        fruit tolerance - stems are supposed to be MORE permissive than fruit
        (natural stem contact is fine), so this number should come down (or
        up) once you calibrate it the same way fruit was calibrated: measure
        a few known-touching and known-separate stem pairs with
        closest_point_markers.py and pick a value between them.

        Returns (rejected: bool, distance: float) - distance replaces the old
        overlap ratio in the return signature. Anything calling
        check_pedicel_pair's stem_overlap_ratio key will need updating to
        read this as a distance instead of a ratio (smaller = closer now,
        not larger = more overlap).
        """
        segments_a = find_stem_segments(pedicel_a)
        segments_b = find_stem_segments(pedicel_b)
        if not segments_a or not segments_b:
            return False, None

        min_dist = None
        for seg_a in segments_a:
            for seg_b in segments_b:
                d = mesh_distance.min_distance(seg_a, seg_b)
                if d is None:
                    continue
                if min_dist is None or d < min_dist:
                    min_dist = d

        if min_dist is None:
            return False, None

        rejected = min_dist < stem_contact_tolerance
        if debug and rejected:
            print(f"  [STEM REJECT] {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}")
            print(f"    min segment distance = {min_dist:.5f}  (tolerance = {stem_contact_tolerance})")

        return rejected, min_dist


    # updated to account for the fruit vs stem collision logic
    def check_pedicel_pair(self, pedicel_a, pedicel_b, debug=False):
        """Combined verdict for one pair. Returns (rejected, reasons dict)."""
        fruit_collision = self.check_fruit_collision(pedicel_a, pedicel_b, debug=debug)
        stem_rejected, stem_distance = self.check_stem_collision(pedicel_a, pedicel_b, debug=debug)
        cross_rejected, cross_distance = self.check_cross_stem_fruit_collision(pedicel_a, pedicel_b, debug=debug)

        rejected = fruit_collision or stem_rejected or cross_rejected
        return rejected, {
            "fruit_collision": fruit_collision,
            "stem_min_distance": stem_distance,
            "cross_min_distance": cross_distance,
        }


