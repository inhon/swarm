from threading import Timer
import time 

class RepeatTimer(Timer):
    def __init__(self, interval, function, args=None, kwargs=None):
        super().__init__(interval, function, args, kwargs)
        self.interval = interval  # Time between executions (in seconds)

    def run(self):
        while True:
            self.function(*self.args, **self.kwargs)
            time.sleep(self.interval)  # Wait for the next interval

# Define a function to be called periodically
def my_function():
    print("Function executed!")

# Create and start the timer
timer = RepeatTimer(5, my_function)  # Call my_function every 5 seconds
timer.start()

# To stop the timer after some time (optional)
# time.sleep(20)
# timer.cancel()  # Stops the periodic execution after 20 seconds
