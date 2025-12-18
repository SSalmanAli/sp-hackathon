# Quickstart: Embedding Pipeline

## Prerequisites

- Python 3.11 or higher
- UV package manager installed
- Cohere API key
- Qdrant instance (local or cloud)

## Setup

### 1. Create Backend Directory
```bash
mkdir backend
cd backend
```

### 2. Initialize Project with UV
```bash
# Create a virtual environment
uv venv

# Activate the virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Create Requirements File
Create `requirements.txt` with the following content:
```txt
requests>=2.31.0
beautifulsoup4>=4.12.2
cohere>=4.0.0
qdrant-client>=1.8.0
python-dotenv>=1.0.0
```

### 4. Install Dependencies
```bash
uv pip install -r requirements.txt
```

### 5. Set Up Environment Variables
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # Optional, for remote Qdrant
QDRANT_API_KEY=your_qdrant_api_key_here  # Optional, for remote Qdrant
QDRANT_HOST=localhost  # Default for local Qdrant
QDRANT_PORT=6333  # Default Qdrant port
```

### 6. Run the Pipeline
```bash
python main.py
```

## Local Qdrant Setup (Optional)

If you want to run Qdrant locally using Docker:

```bash
docker pull qdrant/qdrant
docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

## Configuration

The pipeline can be configured via environment variables in the `.env` file:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL for remote Qdrant instance (optional if using local)
- `QDRANT_API_KEY`: API key for remote Qdrant instance (optional)
- `QDRANT_HOST`: Host for Qdrant connection (default: localhost)
- `QDRANT_PORT`: Port for Qdrant connection (default: 6333)
- `CHUNK_SIZE`: Size of text chunks in characters (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 100)
- `DOCUSSAURUS_URL`: The URL to crawl (default: https://ssalmanali.github.io/sp-hackathon/)

## Usage

The main.py file will execute the complete pipeline:
1. Get all URLs from the specified Docusaurus site
2. Extract text from each URL
3. Chunk the text into manageable pieces
4. Generate embeddings using Cohere
5. Create the 'rag-embedding' collection in Qdrant if it doesn't exist
6. Save the text chunks with embeddings to Qdrant