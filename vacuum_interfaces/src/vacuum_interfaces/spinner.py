import sys
class Spinner():
    def __init__(self):
        self.__blip = "|"
    def spin(self):
        p = { "|": "/",
              "/": "-",
              "-": "\\",
              "\\": "|"}
        self.__blip = p[self.__blip]
        print "\b\b"+self.__blip,
        sys.stdout.flush()
spinner = Spinner()


