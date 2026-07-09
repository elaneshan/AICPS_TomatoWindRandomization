import omni.ext
import carb


class TomatoWindExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        carb.log_info("================================")
        carb.log_info("Hello from AICPS Wind Extension")
        carb.log_info("================================")

    def on_shutdown(self):
        carb.log_info("AICPS Wind Extension shutdown")

