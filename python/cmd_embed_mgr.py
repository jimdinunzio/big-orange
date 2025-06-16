import requests
import pickle

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
            "load map",
            "save map",
            "clear map",
            "clear locations",
            "enable mapping",
            "disable mapping",            
            "take a picture",
            "take my picture",
            "close windows",
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
            "hello",
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
        
        self.commands_embeddings = []
        self.commands_norm = []
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
        data = {"model": model, "input": texts, "keep_alive": 60}
        response = self.session.post(self.ollama_url, json=data)
        return response.json()["embeddings"]

    def dist_embeddings(self, embeddings1, embeddings2):
        return sum((a - b) ** 2 for a, b in zip(embeddings1, embeddings2)) ** 0.5

    def cosine_dist_embeddings(self, embeddings1, embeddings2):
        dot_product = sum(a * b for a, b in zip(embeddings1, embeddings2))
        norm_a = sum(a ** 2 for a in embeddings1) ** 0.5
        norm_b = sum(b ** 2 for b in embeddings2) ** 0.5
        return 1 - (dot_product / (norm_a * norm_b))

    def cosine_dist_embeddings_cmd(self, embedding, cmd_idx):
        dot_product = sum(a * b for a, b in zip(embedding, self.commands_embeddings[cmd_idx]))
        norm_a = sum(a ** 2 for a in embedding) ** 0.5
        norm_b = self.commands_norm[cmd_idx]
        return 1 - (dot_product / (norm_a * norm_b))

    def test_embeddings(self, string1, string2):
        embedding1 = self.get_embedding(string1)
        embedding2 = self.get_embedding(string2)
        distance = self.cosine_dist_embeddings(embedding1, embedding2)
        return distance

    def find_closest_command(self, string, threshold=0.3):
        """Find the closest command to the given string, or return None if no match."""
        embedding = self.get_embedding(string)
        min_distance = float('inf')
        closest_command = None
        for i in range(len(self.commands)):
            distance = self.cosine_dist_embeddings_cmd(embedding, i)
            if distance < min_distance:
                min_distance = distance
                closest_command = self.commands[i]
        if min_distance > threshold:
            return None, min_distance
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
        self.commands_norm = [sum(a ** 2 for a in cmd_emb) ** 0.5 for cmd_emb in self.commands_embeddings]
        return True

    def save_cmds_embeddings(self, filename="commands_embeddings.pkl"):
        cmd_embeddings_dict = {cmd: self.commands_embeddings[i] for i, cmd in enumerate(self.commands)}
        with open(filename, "wb") as f:
            pickle.dump(cmd_embeddings_dict, f)
        return True

    def generate_cmd_embeddings_and_save(self):
        self.commands_embeddings = self.get_embeddings(self.commands)
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