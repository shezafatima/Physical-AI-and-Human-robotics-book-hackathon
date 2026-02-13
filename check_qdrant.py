import os
import sys
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from src.rag import qdrant_rag

def check_qdrant_status():
    print("Checking Qdrant collection status...")

    try:
        # Get collection info
        collection_info = qdrant_rag.client.get_collection(qdrant_rag.collection_name)
        print(f"Collection '{qdrant_rag.collection_name}' exists")
        print(f"Points count: {collection_info.points_count}")

        # Try to get a sample of points
        if collection_info.points_count > 0:
            # Get the first few points to see if they have content
            scroll_result = qdrant_rag.client.scroll(
                collection_name=qdrant_rag.collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )

            points, next_page = scroll_result
            print(f"\nSample of first {len(points)} points:")
            for i, point in enumerate(points):
                print(f"Point {i+1}:")
                print(f"  ID: {point.id}")
                print(f"  Content preview: {point.payload.get('content', '')[:100]}...")
                print(f"  Source: {point.payload.get('metadata', {}).get('source', 'Unknown')}")
                print()
        else:
            print("Collection is empty")

    except Exception as e:
        print(f"Error checking Qdrant collection: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_qdrant_status()