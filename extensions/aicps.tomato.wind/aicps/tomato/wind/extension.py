import omni.ext
import carb


class TomatoWindExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        carb.log_warn("================================")
        carb.log_warn("Hello from AICPS Wind Extension")
        carb.log_warn("================================")

    def on_shutdown(self):
        carb.log_warn("AICPS Wind Extension shutdown")

