#!/usr/bin/env python3
"""
Check if content exists in the Qdrant database
"""

import os
import sys

# Add the src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.rag import qdrant_rag

def check_content():
    print("Checking Qdrant content...")

    try:
        # Get collection info
        collection_info = qdrant_rag.client.get_collection(qdrant_rag.collection_name)
        print(f"Collection '{qdrant_rag.collection_name}' contains {collection_info.points_count} vectors")

        if collection_info.points_count > 0:
            # Get a sample of points to see the content
            scroll_result = qdrant_rag.client.scroll(
                collection_name=qdrant_rag.collection_name,
                limit=5,
                with_payload=True,
                with_vectors=False
            )

            points, _ = scroll_result
            print(f"\nFirst {len(points)} points in the collection:")
            for i, point in enumerate(points):
                print(f"Point {i+1}:")
                print(f"  ID: {point.id}")
                content = point.payload.get('content', '')
                print(f"  Content preview: {content[:200]}...")
                print(f"  Source: {point.payload.get('metadata', {}).get('source', 'Unknown')}")
                print()
        else:
            print("Collection is empty")

    except Exception as e:
        print(f"Error checking content: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    check_content()