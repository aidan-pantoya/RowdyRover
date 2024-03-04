import requests

def check_url_availability(url):
    flag = True
    try:
        response = requests.get(url)
        # Check if the status code is successful (e.g., 2xx)
        if response.status_code // 100 == 2:
            print(f"URL {url} is reachable and accepts requests.")
            flag = False
        else:
            print(f"URL {url} is reachable but encountered an error: {response.status_code}")
    except requests.ConnectionError:
        print(f"Could not connect to URL {url}.")
    except requests.Timeout:
        print(f"Connection to URL {url} timed out.")
    except requests.RequestException as e:
        print(f"An error occurred while connecting to URL {url}: {e}")
    finally:
        return flag

name = str(input("Enter username: "))
#custom_header = {'Text': 'Status'}

# URL of your Flask server
url = f'https://www.rowdy-rover.com/user/{name}/data'

while check_url_availability(url):
    print("Invalid URL")
    name = str(input("Enter username: "))
    url = f'https://www.rowdy-rover.com/user/{name}/data'

# Data to send
with open("temp.txt", 'w') as file:
    file.write("Gay")

# Sending a POST request with data
with open('temp.txt', 'rb') as file:
    files = file  # Open the file in binary mode
response = requests.post(url, files=files)

# Check response
if response.status_code == 200:
    print("Data sent successfully")
else:
    print("Failed to send data", response.status_code)
