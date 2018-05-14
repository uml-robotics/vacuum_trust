from enum_state import gen_enum_class
class ExperimentState(gen_enum_class("ExperimentState", "standby", "ready", "running", "paused", "complete")):
    pass
# def c.is_offline()
# def c.set_offline()
# def c.offline -> "offline"
# c == "offline"   
# c.get()
