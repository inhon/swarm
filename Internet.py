import requests
from datetime import datetime
def checkInternetConnection(drone,file=None):
    url = "https://www.google.com"
    timeout = 10
    try:
        request = requests.get(url, timeout=timeout)
        file.write("At time "+datetime.now().strftime("%H%M%S")+"Connected to the Internet\n")
    except (requests.ConnectionError, requests.Timeout) as exception:
        print("No internet connection. Aborting mission")
        if(file): file.write("At time "+datetime.now().strftime("%H%M%S")+" detected loss of connection. Landing!\n")
        drone.emergencyLand()