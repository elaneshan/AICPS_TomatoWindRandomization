import omni.usd

import plant.pivot_finder as pivot_finder

def main():
    stage = omni.usd.get_context().get_stage()

    pedicel = stage.GetPrimAtPath(
        "/World/Tomato_Cluster_Assembly/Peduncle/Rachis/Pedicel_01")
    segment = pivot_finder.find_segment_a(pedicel)
    print("v2")
    print(segment.GetPath())

