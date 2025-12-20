import os
import requests
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from urllib.parse import urljoin, urlparse
import re
from .embeddings import embedder
from .config.settings import settings


class QdrantRAG:
    def __init__(self, collection_name: str = "coursebook_content"):
        self.collection_name = collection_name
        try:
            # Use environment variables if settings values are None
            qdrant_url = settings.QDRANT_URL or os.getenv("QDRANT_URL")
            qdrant_api_key = settings.QDRANT_API_KEY or os.getenv("QDRANT_API_KEY")

            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                prefer_grpc=False
            )
            self._create_collection_if_not_exists()
            print(f"Connected to Qdrant at {qdrant_url}")
        except Exception as e:
            print(f"Could not connect to Qdrant at {qdrant_url}: {str(e)}")
            print("RAG functionality will be limited until Qdrant is available")
            self.client = None

    def _split_text(self, text: str, chunk_size: int = 1000, overlap: int = 200) -> List[str]:
        """
        Simple text splitter that breaks text into overlapping chunks
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If we're at the end, take the remaining text
            if end >= len(text):
                chunks.append(text[start:])
                break

            # Find a good breaking point (try to break at sentence or word boundary)
            chunk = text[start:end]

            # Look for sentence boundaries to break at
            sentence_break = chunk.rfind('. ', max(0, chunk_size//2), len(chunk))
            if sentence_break != -1:
                actual_end = start + sentence_break + 2
            else:
                # Look for word boundaries
                word_break = chunk.rfind(' ', max(0, chunk_size//2), len(chunk))
                if word_break != -1:
                    actual_end = start + word_break
                else:
                    # Just break at the chunk_size if no good boundary found
                    actual_end = end

            chunks.append(text[start:actual_end])
            start = actual_end - overlap  # Overlap for context continuity

            # Ensure we don't get stuck in an infinite loop
            if start >= len(text) or actual_end <= start:
                if start < len(text):
                    chunks.append(text[start:])
                break

        return chunks

    def _create_collection_if_not_exists(self):
        """Create the collection if it doesn't exist"""
        if self.client is None:
            print("Qdrant client not available. Cannot create collection.")
            return

        try:
            # Try to get collection info to see if it exists
            self.client.get_collection(self.collection_name)
        except:
            # If collection doesn't exist, create it
            vector_size = 1024  # Using 1024 dimensions for Cohere embeddings
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )

    def add_documents(self, documents: List[Dict[str, Any]]):
        """Add documents to the Qdrant collection"""
        if not documents:
            return

        if self.client is None:
            print("Qdrant client not available. Cannot add documents.")
            return

        # Prepare points for insertion
        points = []
        for idx, doc in enumerate(documents):
            content = doc.get("content", "")
            metadata = doc.get("metadata", {})

            # Generate embedding for the content
            embedding = embedder.embed_text(content)

            points.append(
                models.PointStruct(
                    id=idx,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            )

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant documents based on the query"""
        if self.client is None:
            print("Qdrant client not available. Cannot perform search.")
            return []

        query_embedding = embedder.embed_text(query)

        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k
        )

        results = []
        for result in search_results:
            results.append({
                "content": result.payload["content"],
                "metadata": result.payload["metadata"],
                "score": result.score
            })

        return results

    def load_course_content(self, course_path: str):
        """Load course content from markdown files and add to vector store"""
        documents = []

        # Walk through the course content directory
        for root, dirs, files in os.walk(course_path):
            for file in files:
                if file.endswith('.md'):
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r', encoding='utf-8') as f:
                        content = f.read()

                        # Create metadata
                        metadata = {
                            "source": file_path,
                            "title": os.path.splitext(file)[0],
                            "type": "course_content"
                        }

                        # Split content into chunks for better retrieval (simple approach)
                        chunks = self._split_text(content, chunk_size=1000, overlap=200)

                        for chunk in chunks:
                            documents.append({
                                "content": chunk,
                                "metadata": metadata
                            })

        # Add all documents to the vector store
        self.add_documents(documents)
        return len(documents)

    def load_content_from_sitemap(self, sitemap_url: str) -> int:
        """
        Load content from a sitemap URL by extracting all URLs and scraping their content
        """
        try:
            # Fetch the sitemap
            response = requests.get(sitemap_url)
            response.raise_for_status()

            # Parse the sitemap XML
            root = ET.fromstring(response.content)

            # Extract URLs from the sitemap
            urls = []
            for url_element in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url'):
                loc_element = url_element.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
                if loc_element is not None:
                    urls.append(loc_element.text)

            # If no URLs found with namespace, try without namespace
            if not urls:
                for url_element in root.findall('.//url'):
                    loc_element = url_element.find('loc')
                    if loc_element is not None:
                        urls.append(loc_element.text)

            documents = []

            # Process each URL
            for url in urls:
                try:
                    print(f"Processing URL: {url}")
                    response = requests.get(url)
                    response.raise_for_status()

                    # Parse HTML content
                    soup = BeautifulSoup(response.content, 'html.parser')

                    # Remove script and style elements
                    for script in soup(["script", "style"]):
                        script.decompose()

                    # Extract text content
                    text_content = soup.get_text()

                    # Clean up the text
                    lines = (line.strip() for line in text_content.splitlines())
                    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                    text_content = ' '.join(chunk for chunk in chunks if chunk)

                    if text_content.strip():
                        # Create metadata
                        metadata = {
                            "source": url,
                            "title": soup.title.string if soup.title else "No Title",
                            "type": "web_content"
                        }

                        # Split content into chunks for better retrieval (simple approach)
                        chunks = self._split_text(text_content, chunk_size=1000, overlap=200)

                        for chunk in chunks:
                            documents.append({
                                "content": chunk,
                                "metadata": metadata
                            })

                except Exception as e:
                    print(f"Error processing URL {url}: {str(e)}")
                    continue

            # Add all documents to the vector store
            self.add_documents(documents)
            return len(documents)

        except Exception as e:
            print(f"Error loading content from sitemap {sitemap_url}: {str(e)}")
            return 0


# Singleton instance
qdrant_rag = QdrantRAG()