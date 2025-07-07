import requests
import pickle
import numpy as np

class CmdEmbedMgr:
    def __init__(self, ollama_url="http://localhost:11434/api/embed"):
        self.commands = [
            "what is your name",
            "stop moving",
            "who are you",
            "what did you say",
            "goodbye",
            "start listening",
            "stop listening",
            "who are you with",
            "how are you",
            "where are you",
            "where are you going",
            "what time is it",
            "list people you know",
            "list faces you know",
            "go across the room and come back",
            "go recharge",
            "go to your dock",
            "go home",
            "back up",
            "wake up",
            "go to sleep",
            "locate yourself",
            "list your threads",
            "open your eyes",
            "close your eyes",
            "battery",
            "voltage",
            "show map",
            "hide map",
            "load map",
            "save map",
            "clear map",
            "clear locations",
            "enable mapping",
            "disable mapping",            
            "take a picture",
            "take my picture",
            "close pictures",
            "all loaded",
            "all taken",
            "open depth window",
            "close depth window",
            "open rgb window",
            "close rgb window",
            "follow me",
            "stop following me",
            "track me",
            "stop tracking me",
            "look over here",
            "never mind",
            "dance with me",
            "go where I am pointing",
            "list locations",
            "come here",
            "local speech",
            "cloud speech",
            #"enable radar",
            #"disable radar",
            #"open weather chat",
            "enable chat bot",
            "disable chat bot",
        ]
        
        self.session = None
        self.ollama_url = ollama_url
        self.open_ollama_session()

    def __del__(self):
        self.close_ollama_session()

    def open_ollama_session(self):
        self.session = requests.Session()
        self.session.headers.update({'Content-Type': 'application/json'})

    def close_ollama_session(self):
        if self.session:
            self.session.close()
            self.session = None

    def get_embedding(self, text, model="nomic-embed-text"):
        data = {"model": model, "input": text, "keep_alive": 60}
        response = self.session.post(self.ollama_url, json=data)
        return response.json()["embeddings"][0]

    def get_embeddings(self, texts, model="nomic-embed-text"):
        qualified_texts = ["User gave a command to robot: " + text for text in texts]
        data = {"model": model, "input": qualified_texts, "keep_alive": 60}
        response = self.session.post(self.ollama_url, json=data)
        return response.json()["embeddings"]

    def dist_embeddings(self, embeddings1, embeddings2):
        return sum((a - b) ** 2 for a, b in zip(embeddings1, embeddings2)) ** 0.5

    def cosine_dist_embeddings(self, embeddings1, embeddings2):
        embeddings1 = np.array(embeddings1)
        embeddings2 = np.array(embeddings2)
        dot_product = np.dot(embeddings1, embeddings2)
        norm_a = np.linalg.norm(embeddings1)
        norm_b = np.linalg.norm(embeddings2)
        if norm_a == 0 or norm_b == 0:
            return 1.0
        return 1 - (dot_product / (norm_a * norm_b))

    def cosine_dist_embeddings_cmd(self, embedding, cmd_idx):
        embedding = np.array(embedding)
        cmd_embedding = np.array(self.commands_embeddings[cmd_idx])
        dot_product = np.dot(embedding, cmd_embedding)
        norm_a = np.linalg.norm(embedding)
        norm_b = self.commands_norm_np[cmd_idx]
        if norm_a == 0 or norm_b == 0:
            return 1.0
        return 1 - (dot_product / (norm_a * norm_b))

    def test_embeddings(self, string1, string2):
        embedding1 = self.get_embedding(string1)
        embedding2 = self.get_embedding(string2)
        distance = self.cosine_dist_embeddings(embedding1, embedding2)
        return distance

    def find_closest_command(self, string):
        """Find the closest command to the given string."""
        embedding = self.get_embedding("User gave a command to robot: " + string)
        min_distance = float('inf')
        embedding_np = np.array(embedding)
        dot_products = self.commands_embeddings_np @ embedding_np
        norm_a = np.linalg.norm(embedding_np)
        cosine_distances = 1 - (dot_products / (norm_a * self.commands_norm_np))
        min_idx = np.argmin(cosine_distances)
        closest_command = self.commands[min_idx]
        min_distance = cosine_distances[min_idx]
        return closest_command, min_distance

    def load_cmds_embeddings(self, filename="commands_embeddings.pkl"):
        try:
            with open(filename, "rb") as f:
                cmd_embeddings_dict = pickle.load(f)
        except FileNotFoundError:
            print(f"File {filename} not found.")
            return False
        self.commands = list(cmd_embeddings_dict.keys())
        self.commands_embeddings = list(cmd_embeddings_dict.values())
        self.commands_embeddings_np = np.array(self.commands_embeddings)
        self.commands_norm_np = np.linalg.norm(self.commands_embeddings_np, axis=1)
        return True

    def save_cmds_embeddings(self, filename="commands_embeddings.pkl"):
        cmd_embeddings_dict = {cmd: self.commands_embeddings[i] for i, cmd in enumerate(self.commands)}
        with open(filename, "wb") as f:
            pickle.dump(cmd_embeddings_dict, f)
        return True

    def generate_cmd_embeddings_and_save(self):
        print("generating command embeddings")
        self.commands_embeddings = self.get_embeddings(self.commands)
        print("saving command embeddings")
        self.save_cmds_embeddings()

    def analyze_command_distances(self):
        """Analyze pairwise cosine distances between all commands."""
        distances = []
        for i, emb1 in enumerate(self.commands_embeddings):
            for j, emb2 in enumerate(self.commands_embeddings):
                if i != j:
                    dist = self.cosine_dist_embeddings(emb1, emb2)
                    distances.append(dist)
        if distances:
            print(f"Min distance: {min(distances)}")
            print(f"Max distance: {max(distances)}")
            print(f"Mean distance: {sum(distances)/len(distances)}")
            print(f"Median distance: {sorted(distances)[len(distances)//2]}")
        else:
            print("No distances computed.")

if __name__ == "__main__":
    cmd_embed_mgr = CmdEmbedMgr()
    cmd_embed_mgr.generate_cmd_embeddings_and_save()
    cmd_embed_mgr.load_cmds_embeddings()
    cmd_embed_mgr.analyze_command_distances()
    # Test the find_closest_command method
    test_string = "turn on mapping"
    closest_command, distance = cmd_embed_mgr.find_closest_command(test_string)
    print(f"Closest command to '{test_string}': {closest_command} with distance {distance}")