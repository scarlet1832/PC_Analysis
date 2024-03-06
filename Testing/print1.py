import datetime
import time

class print_1:
    def __init__(self):
        self.a = '1226'

    def print_time(self):
        while True:
            current_time = datetime.datetime.now()
            print(self.a, '+', current_time)
            time.sleep(1)

if __name__ == '__main__':
    print_1().print_time()