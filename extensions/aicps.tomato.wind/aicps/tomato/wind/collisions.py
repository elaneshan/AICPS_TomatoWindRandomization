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

    def __init__(self, stage, stem_overlap_threshold=0.15, stem_notable_threshold= 0.02, environment_prim=None):
        self.stage = stage
        self.stem_overlap_threshold = stem_overlap_threshold  # dead code now stems use mesh distance, kept for compat
        self.stem_notable_threshold = stem_notable_threshold  # for baseline_report only, not a rejection threshold
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




    # replaced
    def check_fruit_collision(self, pedicel_a, pedicel_b, contact_tolerance=0.009, debug=False):
        """
        Ground-truth mesh distance instead of a bounding-volume approximation.
        contact_tolerance is the real-world gap (in stage units) below which
        two tomatoes are considered touching/overlapping. Calibrated from
        measured pairs: 02-04 (0.0027) and 06-08 (0.0056) are genuine contact;
        01-03 (0.0641) is a confirmed real gap - tolerance sits well below that.

        debug=True uses the slower provenance-tracking distance calc and
        prints which mesh prim the closest point pair actually came from -
        use this only for the baseline diagnostic pass, not per-candidate
        during randomization.
        """
        tomato_a = find_child_by_prefix(pedicel_a, "tomato")
        tomato_b = find_child_by_prefix(pedicel_b, "tomato")
        if tomato_a is None or tomato_b is None:
            print(f"WARNING: no tomato found on {pedicel_a.prim.GetName()} or {pedicel_b.prim.GetName()}")
            return False

        if debug:
            info = mesh_distance.min_distance_verbose(tomato_a, tomato_b)
            if info is None:
                return False
            distance = info["distance"]
            if distance < contact_tolerance:
                print(f"  [FRUIT REJECT] {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}")
                print(f"    distance = {distance:.5f}  (tolerance = {contact_tolerance})")
                print(f"    closest point A on: {info['mesh_a_path']}  @ {info['point_a']}")
                print(f"    closest point B on: {info['mesh_b_path']}  @ {info['point_b']}")
            return distance < contact_tolerance

        distance = mesh_distance.min_distance(tomato_a, tomato_b)
        if distance is None:
            return False
        return distance < contact_tolerance


    # --- And thread debug through baseline_report so you can flip it on there ---

    def baseline_report(self, rig, debug=False):
        print("\n===== Baseline Collision Report (current pose) =====")
        any_rejected = False
        for i, pedicel_a in enumerate(rig.pedicels):
            for pedicel_b in rig.pedicels[i + 1:]:
                fruit_collision = self.check_fruit_collision(pedicel_a, pedicel_b, debug=debug)
                stem_rejected, stem_distance = self.check_stem_collision(pedicel_a, pedicel_b, debug=debug)
                rejected = fruit_collision or stem_rejected
                info = {"fruit_collision": fruit_collision, "stem_min_distance": stem_distance}


                if rejected:
                    any_rejected = True
                    print(f"  REJECT: {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}  {info}")
                elif stem_distance is not None and stem_distance < self.stem_notable_threshold:
                    print(f"  ok (stem close but not rejected): {pedicel_a.prim.GetName()} <-> {pedicel_b.prim.GetName()}  distance={stem_distance:.5f}")
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


    # --- check_pedicel_pair needs a small update since stem_overlap_ratio is now a distance ---

    def check_pedicel_pair(self, pedicel_a, pedicel_b, debug=False):
        """Combined verdict for one pair. Returns (rejected, reasons dict)."""
        fruit_collision = self.check_fruit_collision(pedicel_a, pedicel_b, debug=debug)
        stem_rejected, stem_distance = self.check_stem_collision(pedicel_a, pedicel_b, debug=debug)

        rejected = fruit_collision or stem_rejected
        return rejected, {
            "fruit_collision": fruit_collision,
            "stem_min_distance": stem_distance,  # renamed from stem_overlap_ratio - now a distance, not a ratio
        }

