import random, math
from pxr import UsdLux, Gf, UsdGeom
from .camera import set_look_at, WORLD_UP  # reuse the same set_look_at + up convention

DOME_LIGHT_PATH = "/World/Lighting/DomeLight"
KEY_LIGHT_PATH = "/World/Lighting/KeyLight"


def create_lighting_rig(stage):
    dome = UsdLux.DomeLight.Define(stage, DOME_LIGHT_PATH)
    dome.CreateIntensityAttr(800.0)
    dome.CreateEnableColorTemperatureAttr(True)
    dome.CreateColorTemperatureAttr(6500.0)

    key = UsdLux.DistantLight.Define(stage, KEY_LIGHT_PATH)
    key.CreateIntensityAttr(3000.0)
    key.CreateEnableColorTemperatureAttr(True)
    key.CreateColorTemperatureAttr(5500.0)
    key.CreateAngleAttr(2.0)  # angular size -> softness of shadow edge
    return dome, key


def randomize_lighting(
    stage,
    dome_intensity_range=(300.0, 1500.0),
    dome_color_temp_range=(3000.0, 6500.0),
    key_intensity_range=(1500.0, 6000.0),
    key_color_temp_range=(3000.0, 6500.0),
    key_azimuth_range=(0.0, 360.0),
    key_elevation_range=(15.0, 80.0),   # keep above horizon-ish, avoid pure silhouette
):
    dome_prim = stage.GetPrimAtPath(DOME_LIGHT_PATH)
    key_prim = stage.GetPrimAtPath(KEY_LIGHT_PATH)
    if not dome_prim.IsValid() or not key_prim.IsValid():
        create_lighting_rig(stage)
        dome_prim = stage.GetPrimAtPath(DOME_LIGHT_PATH)
        key_prim = stage.GetPrimAtPath(KEY_LIGHT_PATH)

    dome = UsdLux.DomeLight(dome_prim)
    dome.GetIntensityAttr().Set(random.uniform(*dome_intensity_range))
    dome.GetColorTemperatureAttr().Set(random.uniform(*dome_color_temp_range))

    key = UsdLux.DistantLight(key_prim)
    key.GetIntensityAttr().Set(random.uniform(*key_intensity_range))
    key.GetColorTemperatureAttr().Set(random.uniform(*key_color_temp_range))

    az = math.radians(random.uniform(*key_azimuth_range))
    el = math.radians(random.uniform(*key_elevation_range))
    # Z-up direction the light shines FROM, same convention as camera azimuth/elevation
    direction = Gf.Vec3d(
        math.cos(el) * math.cos(az),
        math.cos(el) * math.sin(az),
        math.sin(el),
    )
    # DistantLight only cares about orientation, not position - point it
    # from a spot opposite the shine direction, back toward the origin
    set_look_at(UsdGeom.Xformable(key_prim), -direction, Gf.Vec3d(0, 0, 0))

    return {
        "dome_intensity": dome.GetIntensityAttr().Get(),
        "dome_color_temp": dome.GetColorTemperatureAttr().Get(),
        "key_intensity": key.GetIntensityAttr().Get(),
        "key_color_temp": key.GetColorTemperatureAttr().Get(),
        "key_azimuth_deg": math.degrees(az),
        "key_elevation_deg": math.degrees(el),
    }

