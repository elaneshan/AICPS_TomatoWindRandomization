#  given a pedicel, we need to be able to calculate
# the hinge (aka the top of segement_A which is what connects to the rachis and where we want it to pivot)


from pxr import Usd, UsdGeom, Gf, Sdf




class PivotFinder:
    """Computes hinge points for pedicels and leaves."""


    def __init__(self, stage):
        self.stage = stage


        self.bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_],
        )


    def compute_hinge(self, pedicel_data):
        """
        Compute the hinge point for a pedicel.
        Returns a world-space Gf.Vec3d.
        """
        hinge_segment = self._find_hinge_segment(pedicel_data.prim) # using the data models we created in registry!


        # get all the boundaries for that segment
        bounds = self.bbox_cache.ComputeWorldBound(hinge_segment)
        bbox = bounds.ComputeAlignedBox()


        minimum = bbox.GetMin()
        maximum = bbox.GetMax()


        # Top of Segment A (or B in fallback) aka hinge point
        hinge = Gf.Vec3d(
            (minimum[0] + maximum[0]) / 2.0,
            maximum[1],
            (minimum[2] + maximum[2]) / 2.0,
        )


        pedicel_data.hinge = hinge


        return hinge


    def compute_leaf_hinge(self, leaf_prim: Usd.Prim, object_0_path: Sdf.Path = None) -> tuple[Gf.Vec3d, Sdf.Path]:
        """Returns (hinge_point, object_0_path). Pass a cached object_0_path on
        subsequent calls to skip the subtree search."""
        if object_0_path is not None:
            object_0 = self.stage.GetPrimAtPath(object_0_path)
        else:
            object_0 = None
            for desc in Usd.PrimRange(leaf_prim):
                if desc.GetName() == "Object_0":
                    object_0 = desc
                    break

        if object_0 is None or not object_0.IsValid():
            raise ValueError(f"No Object_0 found under {leaf_prim.GetPath()}")

        # ComputeLocalToWorldTransform walks every intermediate GLTF import node
        # correctly -- no manual parent-world * child-local composition here.
        xf = UsdGeom.Xformable(object_0)
        world_matrix = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return world_matrix.ExtractTranslation(), object_0.GetPath()











    # helper to find the segment itself from the pedicel xform
    def _find_hinge_segment(self, pedicel):
        """returns the segment that is attached to the rachis"""
        segment_a = None
        segment_b = None
        for child in pedicel.GetChildren():


            name = child.GetName().lower()


            if name.startswith("segement_a") or name.startswith("segment_a"):
                segment_a = child
          
            if name.startswith("segment_b"):
                segment_b = child


        if segment_a:
            return segment_a
        if segment_b:
            return segment_b


        raise RuntimeError(
            f"Segment A or B not found under {pedicel.GetPath()}"
        )



