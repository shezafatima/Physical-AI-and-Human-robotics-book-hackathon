#!/usr/bin/env python3
"""
Test script to verify the API endpoints are working correctly
"""
import requests
import json
import time

def test_api_endpoints():
    base_url = "http://localhost:8000"

    print("Testing API endpoints...")

    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        print(f"Root endpoint: {response.status_code} - {response.json().get('message', 'No message')}")
    except Exception as e:
        print(f"Root endpoint failed: {e}")

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health endpoint: {response.status_code} - {response.json().get('status', 'No status')}")
    except Exception as e:
        print(f"Health endpoint failed: {e}")

    # Test v1 health endpoint
    try:
        response = requests.get(f"{base_url}/v1/health")
        print(f"V1 Health endpoint: {response.status_code} - {response.json().get('status', 'No status')}")
    except Exception as e:
        print(f"V1 Health endpoint failed: {e}")

    # Test query endpoint with a simple query
    try:
        query_data = {
            "query": "What is Physical AI?",
            "user_id": "test-user",
            "metadata": {
                "context_mode": "full_content",
                "source": "frontend"
            }
        }

        response = requests.post(f"{base_url}/v1/query",
                                json=query_data,
                                headers={"Content-Type": "application/json"})

        if response.status_code == 200:
            result = response.json()
            print(f"Query endpoint: {response.status_code}")
            print(f"Response: {result.get('response', 'No response')[:100]}...")
            print(f"Sources: {len(result.get('sources', []))} sources found")
        else:
            print(f"Query endpoint failed with status {response.status_code}: {response.text}")
    except Exception as e:
        print(f"Query endpoint failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_api_endpoints()