#  given a pedicel, we need to be able to calculate 
# the hinge (aka the top of segement_A which is what connects to the rachis and where we want it to pivot)

from pxr import Usd, UsdGeom, Gf


class PivotFinder:
    """Computes hinge points for pedicels."""

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

        hinge_segment = self._find_hinge_segment(pedicel_data.prim) # using the data models we created in regisrty!

        # get all the boundries for that segment 
        bounds = self.bbox_cache.ComputeWorldBound(hinge_segment)
        bbox = bounds.ComputeAlignedBox()

        minimum = bbox.GetMin()
        maximum = bbox.GetMax()

        # Top of Segment A ( of B in fall back) aka hinge point
        hinge = Gf.Vec3d(
            (minimum[0] + maximum[0]) / 2.0,
            maximum[1],
            (minimum[2] + maximum[2]) / 2.0,
        )

        pedicel_data.hinge = hinge

        return hinge

# helper to find the segment itself from the pedicel xform
    def _find_hinge_segment(self, pedicel):
        """returns the segment that is attached to the rachis"""
        segment_a = None;
        segment_b = None
        for child in pedicel.GetChildren():

            name = child.GetName().lower()

            if name.startswith("segement_a"):
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
