import requests
import json

# Test the API endpoint
def test_query():
    url = "http://localhost:8000/v1/query"

    # Test query about Physical AI
    payload = {
        "query": "What is Physical AI?",
        "top_k": 3
    }

    headers = {
        "Content-Type": "application/json"
    }

    try:
        response = requests.post(url, data=json.dumps(payload), headers=headers)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.json()}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_query()