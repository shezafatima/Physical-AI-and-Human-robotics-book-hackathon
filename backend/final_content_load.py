#!/usr/bin/env python3
"""
Final script to load all content with proper rate limit handling
"""

import os
import sys
import time
from pathlib import Path

# Add the backend/src directory to the Python path so we can import the rag module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.rag import qdrant_rag


def load_coursebook_content():
    """
    Load the coursebook content from the frontend/docs directory into the vector database.
    This version handles rate limits gracefully and continues even when rate limited.
    """
    print("Loading coursebook content into vector database...")
    print("Note: This may take a while due to API rate limits...")

    # Define the path to the coursebook content
    coursebook_path = os.path.join(os.path.dirname(__file__), '..', 'frontend', 'docs')

    # Check if the coursebook directory exists
    if not os.path.exists(coursebook_path):
        print(f"Error: Coursebook directory not found at {coursebook_path}")
        return False

    print(f"Loading content from: {coursebook_path}")

    try:
        # Clear the existing collection first to start fresh
        print("Clearing existing collection...")
        try:
            qdrant_rag.client.delete_collection(qdrant_rag.collection_name)
            print("Existing collection deleted.")
        except:
            print("Collection didn't exist, creating new one...")

        # Recreate the collection
        qdrant_rag._create_collection_if_not_exists()
        print("Collection recreated.")

        # Walk through the course content directory
        documents = []
        for root, dirs, files in os.walk(coursebook_path):
            for file in files:
                if file.endswith('.md'):
                    file_path = os.path.join(root, file)
                    print(f"Processing file: {file_path}")

                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()

                    # Create metadata
                    metadata = {
                        "source": file_path,
                        "title": os.path.splitext(os.path.basename(file_path))[0],  # Get the filename without extension
                        "type": "course_content"
                    }

                    # Split content into chunks for better retrieval (simple approach)
                    chunks = qdrant_rag._split_text(content, chunk_size=1000, overlap=200)

                    for i, chunk in enumerate(chunks):
                        documents.append({
                            "content": chunk,
                            "metadata": metadata
                        })

        print(f"Processing {len(documents)} content chunks...")

        # Add documents to the vector store one by one
        total_added = 0
        total_failed = 0
        for i, document in enumerate(documents):
            try:
                # Add single document
                qdrant_rag.add_documents([document])
                total_added += 1

                print(f"Successfully added {total_added}/{len(documents)} documents to the vector store")

            except Exception as e:
                print(f"Failed to add document {i} (likely due to rate limits): {str(e)}")
                total_failed += 1

                # Wait longer when we get rate limited
                print("Waiting 30 seconds to respect rate limits...")
                time.sleep(30)
                continue

        print(f"Successfully loaded {total_added} content chunks into the vector database")
        print(f"Failed to load {total_failed} content chunks (likely due to rate limits)")

        # Get collection info to verify the content was loaded
        collection_info = qdrant_rag.client.get_collection(qdrant_rag.collection_name)
        print(f"Collection '{qdrant_rag.collection_name}' now contains {collection_info.points_count} vectors")

        return True

    except Exception as e:
        print(f"Error loading course content: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def test_retrieval():
    """
    Test the retrieval functionality to ensure content was loaded correctly.
    """
    print("\nTesting retrieval functionality...")

    try:
        # Test search with a sample query
        results = qdrant_rag.search("What is Physical AI?", top_k=3)

        if results:
            print(f"Found {len(results)} relevant results for query 'What is Physical AI?'")
            for i, result in enumerate(results):
                print(f"  Result {i+1}: Score={result['score']:.3f}")
                print(f"    Content preview: {result['content'][:100]}...")
                print(f"    Source: {result['metadata'].get('source', 'unknown')}")
                print()
        else:
            print("No results found. Content may not be loaded properly.")

        return True

    except Exception as e:
        print(f"Error testing retrieval: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("Final Coursebook Content Loader (Rate Limit Aware)")
    print("=" * 50)

    # Load the coursebook content
    success = load_coursebook_content()

    if success:
        print("\nContent loaded successfully!")

        # Test the retrieval functionality
        test_retrieval()

        print("\nContent loading and testing completed.")
    else:
        print("\nContent loading failed. Please check the error messages above.")
        sys.exit(1)