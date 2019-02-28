#!/usr/bin/env python


class basecontroller(list):

    def __init__(self, comm):
        list.__init__(self)
        self.comm = comm
        self.joystick = None

    def change_joystick(self, cmd):

        self.joystick = cmd
        self.append(cmd)


class controller(basecontroller):

    def __init__(self, comm, *args):
        basecontroller.__init__(self, comm, *args)

        self.create_axes()


# x = controller(1, (2, 3))
# print(x, type(x), repr(x), str(x))

x = basecontroller(1)
print(type(x))
print(type(x.joystick))
# x.change_joystick(3)
# x.change_joystick(27)
# print(len(x))


print('hey win',
      'cool ist tahn')

print('number of specified motors (={})'
      .format(1),
      '!= number of initialized motors (={})'
      .format(2))
