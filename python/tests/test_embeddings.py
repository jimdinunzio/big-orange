import requests

session = None
ollama_url = "http://localhost:11434/api/embed"

def open_ollama_session():
    global session
    # Function to get the embedding of a text using the Nomic API
    session = requests.Session()
    session.headers.update({'Content-Type': 'application/json'})

def close_ollama_session():
    global session
    # Function to close the session
    if session:
        session.close()
        session = None

def get_embedding(text, model="nomic-embed-text"):
    data = {"model": model, "input": text, "keep_alive": 60}
    response = session.post(ollama_url, json=data)
    return response.json()["embeddings"][0]

def cosine_dist_embeddings(embeddings1, embeddings2):
    """Calculate the cosine similarity between two embeddings."""
    dot_product = sum(a * b for a, b in zip(embeddings1, embeddings2))
    norm_a = sum(a ** 2 for a in embeddings1) ** 0.5
    norm_b = sum(b ** 2 for b in embeddings2) ** 0.5
    return 1 - (dot_product / (norm_a * norm_b))

def test_embeddings(string1, string2):
    """Test the embeddings of two strings."""
    embedding1 = get_embedding(string1)
    embedding2 = get_embedding(string2)
    distance = cosine_dist_embeddings(embedding1, embedding2)
    return distance

if __name__ == "__main__":
    # Example strings to test
    import argparse
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Test embeddings of two strings.")
    parser.add_argument("string1", type=str, help="First string to embed")
    parser.add_argument("string2", type=str, help="Second string to embed")
    args = parser.parse_args()
    string1 = args.string1
    string2 = args.string2
    # Check if the strings are not empty
    if not string1 or not string2:
        raise ValueError("Both strings must be provided and not empty.")

    # Run the test
    open_ollama_session()
    print(f"Distance between embeddings: {test_embeddings(string1, string2)}")
