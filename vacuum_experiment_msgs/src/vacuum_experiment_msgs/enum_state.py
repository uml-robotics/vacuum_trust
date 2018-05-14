#    
class BaseClass(object):
    def __init__(self, classtype):
        self._type = classtype


def gen_enum_class(name, *states):
    def __init__(self):
        self._legal_states = states
        self._state = None
        for s in states:
            setattr(self, s, s)
        BaseClass.__init__(self, name)
    def __is_state(self, state):
        return self._state == state
    def __bool__(self):
        return True if not self._state is None else False
    def __repr__(self):
        return "<enum %s>" % (self._state if self._state else "")
    def __str__(self):
        return self._state if self._state else "None"
    def __eq__(self, other):
        return self._state == other
    def unset(self): self._state = None
    def set(self, state):
        if state not in self._legal_states:
            raise Exception("'%s' is not a legal state" % state)
        self._state = state
    def get(self):
        return self._state

    class_dict = {"__init__": __init__, 
                  "__is_state": __is_state,
                  "__str__": __str__,
                  "__repr__": __repr__,
                  "__bool__": __bool__,
                  "__nonzero__": __bool__,
                  "__eq__": __eq__,
                  "set": set, "get": get, "unset": unset}
    for s in states:
        class_dict["is_%s" % s] = lambda x,y=s: x.__is_state(y)
        class_dict["set_%s" % s] = lambda x,y=s: x.set(y)
        class_dict[s.upper()] = s
    return type(name, (BaseClass,), class_dict)

# def ClassFactory(name, argnames, BaseClass=BaseClass): 
#     def __init__(self, **kwargs):
#         for key, value in kwargs.items():
#             if key not in argnames:
#                 raise TypeError("Argument %s not valid for %s" % (key, self.__class__.__name__))
#             setattr(self, key, value)
#         BaseClass.__init__(self, name[:-len("Class")])
#     newclass = type(name, (BaseClass,), {"__init__": __init__})
#     return newclass
# 

# def enum(*sequential, **named):
#     enums = dict(zip(sequential, range(len(sequential))), **named)
#     reverse = dict((value, key) for key, value in enums.iteritems())
#     enums['reverse_mapping'] = reverse
#     return type('Enum', (), enums)



