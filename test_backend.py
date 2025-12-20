#!/usr/bin/env python3
"""
Test script to verify that the backend is running correctly
"""
import requests
import time

def test_backend_endpoints():
    base_url = "http://localhost:8000"

    print("Testing backend endpoints...")

    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            data = response.json()
            print(f"OK Root endpoint: {data['message']}")
        else:
            print(f"X Root endpoint failed with status {response.status_code}")
    except Exception as e:
        print(f"X Root endpoint error: {e}")

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            data = response.json()
            print(f"OK Health endpoint: {data['status']}")
        else:
            print(f"X Health endpoint failed with status {response.status_code}")
    except Exception as e:
        print(f"X Health endpoint error: {e}")

    # Test content ingestion status endpoint
    try:
        response = requests.get(f"{base_url}/content-ingestion/status")
        if response.status_code == 200:
            data = response.json()
            print(f"OK Content ingestion status: {data['message']}")
        else:
            print(f"X Content ingestion status failed with status {response.status_code}")
    except Exception as e:
        print(f"X Content ingestion status error: {e}")

    print("\nBackend is running successfully!")

if __name__ == "__main__":
    # Wait a moment to ensure the server is fully started
    time.sleep(2)
    test_backend_endpoints()