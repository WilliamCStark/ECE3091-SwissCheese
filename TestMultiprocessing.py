import time
from multiprocessing import Process, Pipe, Manager, Queue
from Robot import Robot
from Robot import Motor
import gpiozero
import time


def f(conn):
    i=0
    while (i < 10):
        print("Drive Robot forward")
        time.sleep(1)
        i += 1

def g(q):
    i=0
    while(i < 15):
        print("Get robot sensor info")
        if (i == 3):
            q.put("Terminate")
        time.sleep(1.5)
        i += 1

if __name__ == '__main__':
    q = Queue()
    p1 = Process(target=f,args=(q,))
    p2 = Process(target=g,args=(q,))
    p1.start()
    p2.start()
    exit = False
    while not exit:
        recieved = q.get()
        print(recieved)
        if recieved == "Terminate":
            exit = True
            p1.terminate()
            p1.join()

    print("Thingo terminated")
    p2.join()
    print("Program done")
