"""
Quick test script to verify Qdrant Cloud connection.
"""
import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Force UTF-8 encoding for Windows console
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Load environment variables
load_dotenv()

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

print("=" * 70)
print("TESTING QDRANT CLOUD CONNECTION")
print("=" * 70)
print(f"\nQdrant URL: {qdrant_url}")
print(f"API Key: {'*' * 20}...{qdrant_api_key[-10:] if qdrant_api_key else 'NOT SET'}")

try:
    print("\n[1/3] Creating Qdrant client...")
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )
    print("[OK] Client created successfully")

    print("\n[2/3] Testing connection by fetching collections...")
    collections = client.get_collections()
    print(f"[OK] Connection successful!")
    print(f"  Found {len(collections.collections)} collection(s):")
    for col in collections.collections:
        print(f"    - {col.name}")

    print("\n[3/3] Checking if 'documentation_chunks' collection exists...")
    collection_names = [c.name for c in collections.collections]
    if "documentation_chunks" in collection_names:
        print("[OK] Collection 'documentation_chunks' exists")
    else:
        print("[INFO] Collection 'documentation_chunks' does not exist (will be created on first startup)")

    print("\n" + "=" * 70)
    print("SUCCESS: QDRANT CLOUD CONNECTION WORKS!")
    print("=" * 70)
    print("\nYou can now start your FastAPI server:")
    print("  uvicorn app.main:app --reload --host 0.0.0.0 --port 8000")

except Exception as e:
    print(f"\n[ERROR] CONNECTION FAILED: {e}")
    print("\n" + "=" * 70)
    print("TROUBLESHOOTING:")
    print("=" * 70)
    print("1. Verify QDRANT_URL in .env is correct")
    print("2. Verify QDRANT_API_KEY in .env is correct")
    print("3. Check cluster status at https://cloud.qdrant.io")
    print("4. Ensure your cluster is running (not paused)")
    print("5. Check your internet connection")
    exit(1)
