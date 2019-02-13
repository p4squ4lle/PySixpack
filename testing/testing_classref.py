#!/usr/bin/env python


class basecontroller(list):

    def __init__(self, comm, joystick):
        list.__init__(self)
        self.comm = comm
        self.joystick = joystick



class controller(basecontroller):

    def __init__(self, comm, *args):
        basecontroller.__init__(self, comm, *args)

        self.create_axes()


x = controller(1, (2, 3))
print(x, type(x), repr(x), str(x))
