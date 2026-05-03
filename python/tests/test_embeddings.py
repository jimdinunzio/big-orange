commands = [
    "how are you?",
    "what time is it?",
    "what is your status?",
    "find the bottle and bring it to me",
    "find mr./ms. <name>.",
    "go across the room and come back",
    "go recharge",
    "go to the <location>",
    "go forward",
    "recover localization",
    "open your eyes",
    "close your eyes",
    "load map <name>",
    "save map <name>",
    "clear locations",
    "enable map update",
    "take a picture",
    "take my picture",
    "clear windows",
    "show depth view",
    "show rgb view",
    "hide depth view",
    "hide rgb view",
    "follow me",
    "track me",
    "stop tracking me",
    "look over here",
    "go there"
]

commands_embeddings = []
commands_norm = []

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

def get_embeddings(texts, model="nomic-embed-text"):
    """Get embeddings for a list of texts."""
    data = {"model": model, "input": texts, "keep_alive": 60}
    response = session.post(ollama_url, json=data)
    return response.json()["embeddings"]

def dist_embeddings(embeddings1, embeddings2):
    """Calculate the distance between two embeddings."""
    return sum((a - b) ** 2 for a, b in zip(embeddings1, embeddings2)) ** 0.5

def cosine_dist_embeddings(embeddings1, embeddings2):
    """Calculate the cosine similarity between two embeddings."""
    dot_product = sum(a * b for a, b in zip(embeddings1, embeddings2))
    norm_a = sum(a ** 2 for a in embeddings1) ** 0.5
    norm_b = sum(b ** 2 for b in embeddings2) ** 0.5
    return 1 - (dot_product / (norm_a * norm_b))

def cosine_dist_embeddings_cmd(embedding, cmd):
    """Calculate the cosine similarity between a command and an embedding."""
    dot_product = sum(a * b for a, b in zip(embedding, commands_embeddings[cmd]))
    norm_a = sum(a ** 2 for a in embedding) ** 0.5
    norm_b = commands_norm[cmd]
    return 1 - (dot_product / (norm_a * norm_b))

def test_embeddings(string1, string2):
    """Test the embeddings of two strings."""
    embedding1 = get_embedding(string1)
    embedding2 = get_embedding(string2)
    distance = cosine_dist_embeddings(embedding1, embedding2)
    return distance

def init_cmd_embeddings():
    """Initialize command embeddings."""
    global commands_embeddings
    commands_embeddings = get_embeddings(commands)
    global commands_norm
    commands_norm = [sum(a ** 2 for a in cmd_emb) ** 0.5 for cmd_emb in commands_embeddings]

# find closest command to the given string
def find_closest_command(string):
    """Find the closest command to the given string."""
    embedding = get_embedding(string)
    min_distance = float('inf')
    closest_command = None
    len_cmds = len(commands)
    for i in range(0, len_cmds):
        distance = cosine_dist_embeddings_cmd(embedding, i)
        if distance < min_distance:
            min_distance = distance
            closest_command = commands[i]
    return closest_command, min_distance

def save_cmds_embeddings():
    """Make a dictionary of command embeddings to pickle."""
    import pickle
    cmd_embeddings_dict = {}
    for i, cmd in enumerate(commands):
        cmd_embeddings_dict[cmd] = commands_embeddings[i]
    with open("commands_embeddings.pkl", "wb") as f:
        pickle.dump(cmd_embeddings_dict, f)
    return True

def load_cmds_embeddings():
    """Load a dictionary of command embeddings from pickle."""
    import pickle
    with open("commands_embeddings.pkl", "rb") as f:
        cmd_embeddings_dict = pickle.load(f)
    global commands_embeddings
    commands_embeddings = list(cmd_embeddings_dict.values())
    global commands
    commands = list(cmd_embeddings_dict.keys())
    global commands_norm
    commands_norm = [sum(a ** 2 for a in cmd_emb) ** 0.5 for cmd_emb in commands_embeddings]
    return True

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
    test_embeddings(string1, string2)

    print(f"Distance between embeddings: {test_embeddings(string1, string2)}")
