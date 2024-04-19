import depthai as dai
import time

class Printer(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        print("I'm created - a new Printer object")

    def run(self):
        print("hello world")

class PrinerWithArgs(dai.node.ThreadedHostNode):
    def __init__(self, a=1, b=2, c=3, test="jazsemtest"):
        dai.node.ThreadedHostNode.__init__(self)
        self.a = a
        self.b = b
        self.c = c
        print("I'm created - a new Printer object")
        print("The test is: ", test)
    def run(self):
        print("My args are: ", self.a, self.b, self.c)

p = dai.Pipeline()
p.create(Printer)
a = p.add(Printer())
b = p.add(PrinerWithArgs(2, 3, 4))
p.create(PrinerWithArgs, 10, 20, test="I am a test")
p.start()
p.wait()
