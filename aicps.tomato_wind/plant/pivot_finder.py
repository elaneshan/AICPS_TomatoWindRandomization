# goal of this file is to find the hinge of a "branch like" object
from pxr import UsdGeom

def find_hinge_point(pedicel_prim):
    """Used by rig builder file"""
    pass

# HELPERS

def find_segment_a(pedicel_prim):
    """Returns the Segment_A prim under a pedicel"""
    #print("in finder")
    for child in pedicel_prim.GetChildren():
        name = child.GetName()
        #print(repr(name)) #this print statement is not showing up
        if name.lower().startswith("segement_a"): # i miss typed it soo now im just gonna leave as is...
            return child
    raise RuntimeError(f"No segment_a found in {pedicel_prim.GetPath()}")



def find_mesh(segment_prim):
    """Return the mesh contained within Segment_A"""
    if segment_prim.IsA(UsdGeom.Mesh):
        return segment_prim
    for child in segment_prim.GetChildern():
        if child.IsA(UsdGeom.Mesh):
            return child
        
    raise RuntimeError(f"No mesh found inside {segment_prim.GetPath()}")

def compute_bounds(mesh):
    """Gets the local and global bounds for a given mesh"""
    pass

def choose_hinge_point(mesh):
    """Figure out which end of the segment is connected to the rachis"""
    pass
