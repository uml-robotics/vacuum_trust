from enum_state import gen_enum_class
class ControllerState(gen_enum_class("ControllerState", "offline", "standby", "manual", "ready", "running","paused")):
    pass
# def c.is_offline()
# def c.set_offline()
# def c.offline -> "offline"
# c == "offline"   
# c.get()
